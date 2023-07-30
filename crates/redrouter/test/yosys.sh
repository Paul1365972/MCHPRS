#!/bin/sh

LIBRARIES=redstone_std.lib
LIBRARIES_V=redstone_std.v
MEM_LIBRARIES=redstone_mem.memlib

OUTFILE=$(basename -s .v $1)

yosys -p "
read_verilog $1;
# read_verilog -lib ${LIBRARIES_V}
hierarchy -check -auto-top;
show -format svg -prefix ${OUTFILE}.1 -stretch -width;
proc; opt; fsm; opt; memory -nomap; opt; memory_libmap -lib ${MEM_LIBRARIES}; opt -full;
show -format svg -prefix ${OUTFILE}.2 -stretch -width;
extract -map extract.v; opt -full;
# techmap -map mux.v -D MIN_WIDTH=8; opt -full;
show -format svg -prefix ${OUTFILE}.3 -stretch -width;
techmap; opt -full;
show -format svg -prefix ${OUTFILE}.4 -stretch -width;
dfflibmap -liberty ${LIBRARIES}; opt -full;
show -format svg -prefix ${OUTFILE}.5 -stretch -width;
abc -liberty ${LIBRARIES} -dff; opt -full;
show -format svg -prefix ${OUTFILE}.6 -stretch -width;
check; clean; write_blif ${OUTFILE}.blif; write_json ${OUTFILE}.json;
exec -- rm -f *.dot;
exec -- mv -f *.svg ./viz/;
# Cool commands to add: flatten?; wreduce; hilomap;
"

exit 0
echo "UNREACHABLE P 1"

yosys -p "
read_verilog $1;
show -format svg -prefix ${OUTFILE}.0 -stretch -width;
hierarchy -check -auto-top;
show -format svg -prefix ${OUTFILE}.1 -stretch -width;
proc; opt; fsm; opt; memory; opt;
show -format svg -prefix ${OUTFILE}.2 -stretch -width;
techmap; opt;
show -format svg -prefix ${OUTFILE}.3 -stretch -width;
dfflibmap -liberty ${LIBRARY};
show -format svg -prefix ${OUTFILE}.4 -stretch -width;
abc -liberty ${LIBRARY} -dff;
check;
show -format svg -prefix ${OUTFILE}.5 -stretch -width;
clean; write_blif ${OUTFILE}.blif; write_json ${OUTFILE}.json;
exec -- mv -f *.svg *.dot ./viz/;
"

echo "UNREACHABLE P 2"
