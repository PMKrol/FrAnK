#!/usr/bin/env bash
set -euo pipefail

# === KONFIGURACJA ===
LINK_NAME="/dev/ttyEMU"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MSG_DIR="$SCRIPT_DIR/msgs"

# === SPRZĄTANIE ===
cleanup() {
    echo
    echo "[*] Sprzątanie..."
    if [ -n "${SENDER_PID:-}" ] && ps -p "$SENDER_PID" &>/dev/null; then
        kill "$SENDER_PID" 2>/dev/null || true
    fi
    if [ -n "${SOCAT_PID:-}" ] && ps -p "$SOCAT_PID" &>/dev/null; then
        kill "$SOCAT_PID" 2>/dev/null || true
    fi
    [ -L "$LINK_NAME" ] && sudo rm -f "$LINK_NAME"
    echo "[*] Zakończono."
}
trap cleanup EXIT INT TERM

# === WYMAGANE KOMENDY ===
for cmd in socat shuf find xargs; do
    if ! command -v "$cmd" &>/dev/null; then
        echo "[!] Brakuje komendy: $cmd — zainstaluj ją!"
        exit 1
    fi
done

# === KATALOG Z WIADOMOŚCIAMI ===
mkdir -p "$MSG_DIR"
if [ -z "$(find "$MSG_DIR" -type f | head -n1)" ]; then
    echo "[*] Katalog msgs/ pusty — dodaję przykładowe wiadomości..."
    echo "Test 1" > "$MSG_DIR/test1.txt"
    echo "Test 2" > "$MSG_DIR/test2.txt"
    echo "Test 3 - $(date)" > "$MSG_DIR/test3.txt"
fi

# === URUCHOM SOCAT I PRZECHWYĆ PTY ===
echo "[*] Uruchamiam socat..."

# socat wypisze 2 linie: "PTY is /dev/pts/X" -> przechwycimy je z potoku
exec 3< <(socat -d -d pty,raw,echo=0 pty,raw,echo=0 2>&1)
read -r LINE1 <&3
read -r LINE2 <&3

PTY1=$(echo "$LINE1" | grep -o '/dev/pts/[0-9]\+')
PTY2=$(echo "$LINE2" | grep -o '/dev/pts/[0-9]\+')

if [ -z "$PTY1" ] || [ -z "$PTY2" ]; then
    echo "[!] Nie udało się odczytać PTY z socat:"
    echo "$LINE1"
    echo "$LINE2"
    exit 1
fi

# resztę socat zostawiamy w tle (kanał 3), zapisujemy PID
SOCAT_PID=$(pgrep -f "socat -d -d pty" | head -n1)

# === STWÓRZ ALIAS /dev/ttyEMU ===
if [ -L "$LINK_NAME" ]; then
    sudo rm -f "$LINK_NAME"
fi

echo "[*] Tworzę alias: $LINK_NAME → $PTY2"
sudo ln -s "$PTY2" "$LINK_NAME"

# === PĘTLA WYSYŁAJĄCA CO 1s ===
send_loop() {
    while true; do
        file=$(find "$MSG_DIR" -type f | shuf -n1)
        [ -f "$file" ] || continue
        cat "$file" > "$PTY1"
        echo "" > "$PTY1"
        echo "[TX] $(basename "$file") → $PTY1"
        sleep 1
    done
}

send_loop &
SENDER_PID=$!

# === INFO ===
echo
echo "=== Emulator RS działa ==="
echo " → wysyła przez : $PTY1"
echo " → odbieraj z   : $PTY2"
echo " → alias        : $LINK_NAME"
echo " → katalog msgs : $MSG_DIR"
echo
echo "💡 Odbiór wiadomości: cat $LINK_NAME"
echo "   lub: screen $LINK_NAME 9600"
echo

wait "$SENDER_PID"
