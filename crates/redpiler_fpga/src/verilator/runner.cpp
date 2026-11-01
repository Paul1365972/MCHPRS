#include "Vcompiled_circuit.h"
#include "verilated.h"
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <type_traits>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

uint64_t read_integer(size_t bytes) {
    uint64_t value = 0;
    for (size_t index = 0; index < bytes; ++index) {
        const int byte = std::cin.get();
        if (byte == EOF) throw std::runtime_error("truncated request");
        value |= uint64_t(uint8_t(byte)) << (index * 8);
    }
    return value;
}

void write_integer(uint64_t value, size_t bytes) {
    for (size_t index = 0; index < bytes; ++index) {
        std::cout.put(char(value >> (index * 8)));
    }
}

template <typename Bits>
uint32_t word(const Bits& bits, size_t index) {
    if constexpr (std::is_integral_v<Bits>) {
        return uint32_t(uint64_t(bits) >> (32 * index));
    } else {
        return bits[index];
    }
}

class Runner {
    VerilatedContext context;
    Vcompiled_circuit circuit{&context};
    std::chrono::steady_clock::time_point progress = std::chrono::steady_clock::now();

    void cycle() {
        circuit.clock = 0;
        circuit.eval();
        context.timeInc(1);
        circuit.clock = 1;
        circuit.eval();
        context.timeInc(1);
        if (context.gotFinish()) throw std::runtime_error("RTL stopped simulation");
        const auto now = std::chrono::steady_clock::now();
        if (now - progress >= std::chrono::seconds(1)) {
            std::cout.write("MCHG", 4).flush();
            if (!std::cout) throw std::runtime_error("progress pipe failed");
            progress = now;
        }
    }

    void request() {
        if (circuit.busy) throw std::runtime_error("RTL was busy before request");
        cycle();
        circuit.set_input = 0;
        if (circuit.busy) throw std::runtime_error("RTL request did not complete in one clock");
    }

public:
    Runner() {
        circuit.clock = circuit.set_input = circuit.run = 0;
        circuit.input_index = circuit.input_value = 0;
        circuit.tick_limit = 0;
        circuit.eval();
    }

    ~Runner() { circuit.final(); }

    void input(uint32_t index, bool value) {
        if (index >= MCHPRS_NODE_COUNT) throw std::runtime_error("input index out of bounds");
        const uint64_t ticks = circuit.tick_count;
        circuit.input_index = index;
        circuit.input_value = value;
        circuit.set_input = 1;
        request();
        if (circuit.tick_count != ticks) throw std::runtime_error("input changed RTL tick count");
    }

    void step(uint64_t ticks) {
        if (ticks > std::numeric_limits<uint64_t>::max() - circuit.tick_count) {
            throw std::runtime_error("tick count overflow");
        }
        const uint64_t target = circuit.tick_count + ticks;
        circuit.tick_limit = target;
        circuit.run = 1;
        while (circuit.tick_count != target || circuit.busy) {
            if (circuit.tick_count > target) throw std::runtime_error("RTL ran past the target tick");
            cycle();
        }
        circuit.run = 0;
    }

    void snapshot() {
        std::cout.write("MCHP", 4);
        write_integer(circuit.tick_count, 8);
        for (size_t index = 0; index < MCHPRS_STATE_BYTES; ++index) {
            std::cout.put(char(word(circuit.states, index / 4) >> (index % 4 * 8)));
        }
        std::cout.flush();
        if (!std::cout) throw std::runtime_error("snapshot pipe failed");
    }
};

int main() {
    try {
#ifdef _WIN32
        if (_setmode(_fileno(stdin), _O_BINARY) == -1 ||
            _setmode(_fileno(stdout), _O_BINARY) == -1) {
            throw std::runtime_error("binary pipe mode failed");
        }
#endif
        Runner runner;
        runner.snapshot();
        for (int command; (command = std::cin.get()) != EOF;) {
            if (command == 'I') {
                const uint32_t index = read_integer(4);
                const bool value = read_integer(1) != 0;
                runner.input(index, value);
            } else if (command == 'S') {
                runner.step(read_integer(8));
            } else {
                throw std::runtime_error("unknown request");
            }
            runner.snapshot();
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
