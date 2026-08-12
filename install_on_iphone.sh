#!/usr/bin/env bash
set -u

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sideloader_path="$(command -v sideloader 2>/dev/null || true)"
if [[ -z "$sideloader_path" ]]; then
    sideloader_path="/home/jonasn/.local/bin/sideloader"
fi

if [[ "$#" -gt 0 ]]; then
    ipa_path="$1"
else
    ipa_path=""
    for candidate in "$project_dir"/build/install-*/build/LimbVolScanner.ipa; do
        [[ -f "$candidate" ]] || continue
        if [[ -z "$ipa_path" || "$candidate" -nt "$ipa_path" ]]; then
            ipa_path="$candidate"
        fi
    done
fi

if [[ ! -x "$sideloader_path" ]]; then
    printf 'Sideloader was not found at %s\n' "$sideloader_path" >&2
    exit 1
fi

if [[ -z "$ipa_path" || ! -f "$ipa_path" ]]; then
    printf 'No packaged LimbVolScanner IPA was found under %s/build.\n' \
        "$project_dir" >&2
    printf 'Pass an IPA path as the first argument after downloading a CI build.\n' >&2
    exit 1
fi

mapfile -t connected_devices < <(idevice_id -l)
if [[ "${#connected_devices[@]}" -ne 1 ]]; then
    printf 'Expected exactly one connected iPhone, found %s.\n' \
        "${#connected_devices[@]}" >&2
    exit 1
fi

device_udid="${connected_devices[0]}"
if ! idevicepair validate -u "$device_udid"; then
    printf 'The iPhone is not paired. Unlock it and accept Trust if prompted.\n' >&2
    exit 1
fi

printf '\nInstalling the selected LimbVolScanner IPA on iPhone %s.\n' "$device_udid"
printf 'Enter the Apple ID, password, and temporary 2FA code only at the prompts below.\n'
printf 'The script does not save them or pass them as command-line arguments.\n\n'

"$sideloader_path" install -i --udid "$device_udid" "$ipa_path"
install_status=$?

if [[ "$install_status" -eq 0 ]]; then
    printf '\nInstallation completed. You may close this window.\n'
else
    printf '\nInstallation failed with status %s. Leave this window open for diagnosis.\n' \
        "$install_status" >&2
fi

printf 'Press Enter to close.\n'
read -r
exit "$install_status"
