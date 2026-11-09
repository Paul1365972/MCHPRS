# Adding a block

This document explains the process of adding a new block to MCHPRS.

### Editing the [gen_info.yaml](../mc_data/gen_info.yaml) file

Blocks and items in MCHPRS are defined in the [gen_info.yaml](../mc_data/gen_info.yaml). See the comments in this file for information on how to define a block and its corresponding item.

Add the block under `blocks`, by its name in `blocks.json`, with its attributes. The comments at the top of `gen_info.yaml` describe the attributes.

An item places a block only if the block has the attribute `simple_item` or `item`, or if the item is listed under `block_items`, which is for an item named differently from its block.

Some attributes need code in [`crates/blocks/src/blocks`](../crates/blocks/src/blocks):

| Attribute | Needs |
| --- | --- |
| `prop_struct`, `prop_struct:<Name>` | A struct with a public field per property, named like the property, `ty` for `type` |
| `complex_transform` | `BlockTransform` for that struct |
| `maybe_conductor`, `maybe_transparent` | An arm in `Block::is_conductor_dynamic` or `Block::is_transparent_dynamic` |

### Properties

The generator types each property by its list of values in `blocks.json`:

| Values | Type |
| --- | --- |
| `true`, `false` | `bool` |
| `0` to `6`, `0` to `8`, `0` to `15`, `0` to `24`, `1` to `3`, `1` to `4` | `u8`, or `SignalStrength` for a property `power` with 16 values |
| Any other list | The entry of `prop_types` in `gen_info.yaml` with the same values in the same order |

The generator panics with `could not find prop type` on a list that has no type. Another integer range is added in `main` of the generator.

A new entry of `prop_types` needs an enum of the same name in `crates/blocks`:

- Its variants are the values in PascalCase, in the order of the entry. In another order the block state ids are wrong, and nothing reports it.
- It implements `BlockTransform`, unless only blocks with `complex_transform` use it. `noop_block_transform!` implements it for a type that rotating and flipping leave unchanged.

### Implementing placement logic for complex blocks

If a block has properties or otherwise has complex placement logic, you cannot use the `simple_item` attribute in `gen_info.yaml`. Therefore, the placement logic has to be implemented by hand in the [`core::interaction`](../crates/core/src/interaction.rs) module. Add a match arm for your `Item` variant in the `get_state_for_placement` function. See the other item to block mappings for an example.

### Running the block data generator

Running the block data generator will produce the [generated.rs](../crates/blocks/src/generated.rs) file containing the `Block` and `Item` enums.
It also produces [registries.rs](../crates/blocks/src/registries.rs) with the protocol ids of the registries that [gen_info.yaml](../mc_data/gen_info.yaml) lists under `registries`.

| File in `mc_data` | The generator reads from it |
| --- | --- |
| `gen_info.yaml` | The blocks of MCHPRS and their attributes |
| `blocks.json` | The properties and state ids of each of these blocks |
| `registries.json` | The protocol ids. Every entry of `minecraft:item` becomes a variant of `Item`. |
| `items.json` | The stack size of each item |

```sh
cd crates/block_data_gen
cargo run
```

The generator finds its files relative to this directory. It formats its output with `rustfmt`, which must be on the `PATH`.
