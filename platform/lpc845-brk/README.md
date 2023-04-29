# LPC845-BRK Breakout Board for RJtag

See [here](../../docs/lpc845-brk.md) for more in-depth documentation.

## Manual Debugging

To manually feed data in: 

```
stty -F /dev/ttyACM0 raw && stty -F /dev/ttyACM0 -echo -echoe -echok && stty -F /dev/ttyACM0 115200
```

On one terminal:
```
cat /dev/ttyACM0 | hexdump -v -e '/1 "%02X "' -e '/1 "%c\n"'
```

On another:
```
printf '\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0' > /dev/ttyACM0
```