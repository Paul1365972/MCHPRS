# rilc

It is a development tool:

```sh
cargo run --release -p rilc -- <command>
```

| Command | Effect |
| --- | --- |
| `test <path>` | Run the [tests](RIL.md#tests) of a RIL file, or of every RIL file under a directory. |

## Tests

`rilc test crates/redpiler/ril_tests` runs the tests of the passes. `--update` replaces the expected circuit of each failing test with its result.
