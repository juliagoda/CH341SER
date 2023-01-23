# Recipe for Ubuntu systems

### Install dependencies:

```bash
apt install linux-headers-$(uname -r)
```

### Build and load / unload

```bash
make

sudo make load

ls /dev/ttyUSB*

sudo make unload
```

### Troubleshooting

If you see `usbfs: interface 0 claimed by ch34x while 'brltty' sets config #1` in `sudo demsg`:

```bash
for f in /usr/lib/udev/rules.d/*brltty*.rules; do
    sudo ln -s /dev/null "/etc/udev/rules.d/$(basename "$f")";
done

sudo udevadm control --reload-rules

sudo systemctl mask brltty.path

sudo dmesg
```
