# Example PTX

Simple example of a PTX node.

```
> cargo embed --release  --features 52832
```

Replace <52832> according to the PTX node architecture.

To document

```
> cargo doc --bins --features 52832
```


## LSM6

| DW      | MCU   | LSM6 |
| ------- | ----- | ---- |
| M_PIN13 | P0.27 | CS   |
| M_PIN6  | P0.12 | SCK  |
| M_PIN14 | P0.29 | MOSI |
| M_PIN16 | P0.23 | MISO |

## Resources

- https://www.st.com/en/mems-and-sensors/lsm6ds3.html
 