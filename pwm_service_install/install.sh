SCRIPT_DIR=$(dirname "$0")

echo "[+] Installing pwm_enabler.service"
sudo cp $SCRIPT_DIR/pwm_enable.service /etc/systemd/system/

echo "[+] Installing pwm_enable.sh"
sudo cp $SCRIPT_DIR/pwm_enable.sh /usr/local/bin/

echo "[+] Enabling pwm_enable.service"
sudo systemctl enable pwm_enable.service

echo "[+] Starting pwm_enable.service"
sudo systemctl start pwm_enable.service

echo "[+] Done"
