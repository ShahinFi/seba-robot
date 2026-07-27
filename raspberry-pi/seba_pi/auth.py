"""Local web authentication for the Raspberry Pi robot server."""

import base64
import hashlib
import hmac
import os
import secrets
import threading
import time
from http import cookies


HASH_ALGORITHM = "pbkdf2_sha256"
HASH_ITERATIONS = 240000
SESSION_COOKIE = "seba_session"
SESSION_TTL_S = 12 * 60 * 60
LOGIN_WINDOW_S = 60
LOGIN_MAX_FAILURES = 5


def load_env_file(path):
    """Load KEY=value lines without overriding existing environment values."""

    if not path or not os.path.isfile(path):
        return

    with open(path, "r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue

            key, value = line.split("=", 1)
            key = key.strip()
            value = value.strip().strip('"').strip("'")
            if key and key not in os.environ:
                os.environ[key] = value


def hash_password(password):
    """Return a PBKDF2 password hash suitable for local config files."""

    salt = secrets.token_bytes(16)
    digest = hashlib.pbkdf2_hmac(
        "sha256",
        password.encode("utf-8"),
        salt,
        HASH_ITERATIONS,
    )
    return "$".join([
        HASH_ALGORITHM,
        str(HASH_ITERATIONS),
        base64.urlsafe_b64encode(salt).decode("ascii"),
        base64.urlsafe_b64encode(digest).decode("ascii"),
    ])


def _verify_password(password, stored_hash):
    try:
        algorithm, iterations, salt_b64, digest_b64 = stored_hash.split("$", 3)
        if algorithm != HASH_ALGORITHM:
            return False

        salt = base64.urlsafe_b64decode(salt_b64.encode("ascii"))
        expected = base64.urlsafe_b64decode(digest_b64.encode("ascii"))
        actual = hashlib.pbkdf2_hmac(
            "sha256",
            password.encode("utf-8"),
            salt,
            int(iterations),
        )
        return hmac.compare_digest(actual, expected)
    except Exception:
        return False


class AuthManager:
    """Issue local sessions and enforce operator/engineer access."""

    def __init__(self):
        self.operator_hash = os.environ.get("SEBA_OPERATOR_PASSWORD_HASH", "")
        self.engineer_hash = os.environ.get(
            "SEBA_ENGINEER_PASSWORD_HASH",
            self.operator_hash,
        )
        self.secret = os.environ.get("SEBA_SESSION_SECRET", "")
        self.sessions = {}
        self.failed_logins = {}
        self._lock = threading.Lock()

    @property
    def configured(self):
        return bool(self.operator_hash and self.secret)

    def status_for_headers(self, headers):
        session = self.session_from_headers(headers)
        return {
            "configured": self.configured,
            "authenticated": session is not None,
            "role": session["role"] if session is not None else "none",
            "csrf": session["csrf"] if session is not None else "",
        }

    def login(self, role, password, client_id="local"):
        if not self.configured:
            raise PermissionError("web authentication is not configured")

        with self._lock:
            self._check_login_limit(client_id)

        if role == "engineer":
            stored_hash = self.engineer_hash
        elif role == "operator":
            stored_hash = self.operator_hash
        else:
            with self._lock:
                self._record_failed_login(client_id)
            raise PermissionError("unknown login role")

        if not stored_hash or not _verify_password(password, stored_hash):
            with self._lock:
                self._record_failed_login(client_id)
            raise PermissionError("invalid credentials")

        token = secrets.token_urlsafe(32)
        cookie_token = self._sign_token(token)
        session = {
            "role": role,
            "csrf": secrets.token_urlsafe(24),
            "expires": time.monotonic() + SESSION_TTL_S,
        }

        with self._lock:
            self._prune_expired_sessions()
            self.failed_logins.pop(client_id, None)
            self.sessions[token] = session

        return cookie_token, session

    def logout(self, headers):
        token = self._verified_token_from_headers(headers)
        if not token:
            return

        with self._lock:
            self.sessions.pop(token, None)

    def session_from_headers(self, headers):
        if not self.configured:
            return None

        token = self._verified_token_from_headers(headers)
        with self._lock:
            session = self.sessions.get(token)
            if session is None:
                return None

            if session["expires"] < time.monotonic():
                self.sessions.pop(token, None)
                return None

            return dict(session)

    def require_role(self, headers, minimum_role):
        session = self.session_from_headers(headers)
        if session is None:
            raise PermissionError("login required")

        if minimum_role == "engineer" and session["role"] != "engineer":
            raise PermissionError("engineer access required")

        return session

    def require_csrf(self, headers, session):
        token = headers.get("X-SEBA-CSRF", "")
        if not hmac.compare_digest(token, session["csrf"]):
            raise PermissionError("invalid csrf token")

    def _verified_token_from_headers(self, headers):
        raw_cookie = headers.get("Cookie", "")
        if not raw_cookie:
            return ""

        jar = cookies.SimpleCookie()
        try:
            jar.load(raw_cookie)
        except cookies.CookieError:
            return ""

        morsel = jar.get(SESSION_COOKIE)
        if morsel is None:
            return ""

        try:
            token, signature = morsel.value.split(".", 1)
        except ValueError:
            return ""

        if not hmac.compare_digest(signature, self._signature(token)):
            return ""

        return token

    def _sign_token(self, token):
        return f"{token}.{self._signature(token)}"

    def _signature(self, token):
        return hmac.new(
            self.secret.encode("utf-8"),
            token.encode("utf-8"),
            hashlib.sha256,
        ).hexdigest()

    def _check_login_limit(self, client_id):
        now = time.monotonic()
        attempts = [
            timestamp
            for timestamp in self.failed_logins.get(client_id, [])
            if now - timestamp < LOGIN_WINDOW_S
        ]
        self.failed_logins[client_id] = attempts

        if len(attempts) >= LOGIN_MAX_FAILURES:
            raise PermissionError("too many login attempts")

    def _record_failed_login(self, client_id):
        attempts = self.failed_logins.setdefault(client_id, [])
        attempts.append(time.monotonic())

    def _prune_expired_sessions(self):
        now = time.monotonic()
        expired = [
            token
            for token, session in self.sessions.items()
            if session["expires"] < now
        ]

        for token in expired:
            self.sessions.pop(token, None)


def build_session_cookie(token, max_age=SESSION_TTL_S):
    cookie = cookies.SimpleCookie()
    cookie[SESSION_COOKIE] = token
    cookie[SESSION_COOKIE]["path"] = "/"
    cookie[SESSION_COOKIE]["max-age"] = str(max_age)
    cookie[SESSION_COOKIE]["httponly"] = True
    cookie[SESSION_COOKIE]["samesite"] = "Strict"
    return cookie.output(header="").strip()


def build_clear_cookie():
    return build_session_cookie("", max_age=0)
