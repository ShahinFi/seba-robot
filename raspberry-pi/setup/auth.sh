#!/usr/bin/env bash

set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
. "$SCRIPT_DIR/common.sh"

CONFIG_FILE="$RASPBERRY_PI_DIR/local_config.env"

require_command "$REPO_DIR/.venv/bin/python3"

read_password_twice() {
    prompt="$1"

    first="$(prompt_secret _UNUSED "$prompt")"
    second="$(prompt_secret _UNUSED "Confirm $prompt")"

    [ "$first" = "$second" ] ||
        fail "Passwords did not match."

    [ "${#first}" -ge 8 ] ||
        fail "Password must be at least 8 characters."

    printf '%s' "$first"
}

if is_noninteractive; then
    operator_password="${SEBA_OPERATOR_PASSWORD:-}"
    [ -n "$operator_password" ] ||
        fail "SEBA_OPERATOR_PASSWORD must be set in non-interactive mode."
    [ "${#operator_password}" -ge 8 ] ||
        fail "SEBA_OPERATOR_PASSWORD must be at least 8 characters."
else
    operator_password="$(read_password_twice "Operator web password")"
fi

if is_noninteractive; then
    engineer_password="${SEBA_ENGINEER_PASSWORD:-$operator_password}"
    [ "${#engineer_password}" -ge 8 ] ||
        fail "SEBA_ENGINEER_PASSWORD must be at least 8 characters."
else
    printf 'Use same password for engineer access? [Y/n]: ' >&2
    read -r same_engineer
    case "$same_engineer" in
        n|N|no|NO|No)
            engineer_password="$(read_password_twice "Engineer web password")"
            ;;
        *)
            engineer_password="$operator_password"
            ;;
    esac
fi

operator_hash="$(
    printf '%s' "$operator_password" |
        PYTHONPATH="$RASPBERRY_PI_DIR" \
            "$REPO_DIR/.venv/bin/python3" -m seba_pi.auth_hash --stdin
)"
engineer_hash="$(
    printf '%s' "$engineer_password" |
        PYTHONPATH="$RASPBERRY_PI_DIR" \
            "$REPO_DIR/.venv/bin/python3" -m seba_pi.auth_hash --stdin
)"
session_secret="$(
    "$REPO_DIR/.venv/bin/python3" -c 'import secrets; print(secrets.token_urlsafe(48))'
)"

tmp_file="$(mktemp)"
{
    printf 'SEBA_OPERATOR_PASSWORD_HASH=%s\n' "$operator_hash"
    printf 'SEBA_ENGINEER_PASSWORD_HASH=%s\n' "$engineer_hash"
    printf 'SEBA_SESSION_SECRET=%s\n' "$session_secret"
} > "$tmp_file"

if [ -f "$CONFIG_FILE" ] && ! cmp -s "$CONFIG_FILE" "$tmp_file"; then
    backup="${CONFIG_FILE}.seba-backup-$(timestamp)"
    cp "$CONFIG_FILE" "$backup"
    chmod 600 "$backup"
    info "Backup: $backup"
fi

install -m 0600 "$tmp_file" "$CONFIG_FILE"
rm -f "$tmp_file"

info "Web authentication config written: $CONFIG_FILE"
record_install_state
