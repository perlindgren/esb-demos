# Example PRX

Simple example of a PRX node.

```
> cargo embed --release  --features 52840
```

Replace <52840> according to the PRX node architecture.

To document
```
> cargo doc --bins --features 52840
```

## LSM6DS3, Connections

| Function  | Pin   |
|-----------| ----- |
| CS        | p0.21 |
| MISO      | p0.22 | 
| MOSI      | p0.23 |
| CLK       | p0.24 |
- INT1
- INT2


