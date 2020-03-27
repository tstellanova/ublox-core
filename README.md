## ublox-core
An embedded hal driver for communicating with Ublox position 
devices (M8 or later)

## Status

This is work-in-progress

- [x] Basic support for USART reads
- [ ] USB support
- [ ] SPI support
- [ ] I2C support


## Examples

Build example for eg Durandal stm32h743 board target:
```
cargo build --example serial --target thumbv7em-none-eabihf 
```


## License

BSD-3-Clause, see `LICENSE` file.
 
