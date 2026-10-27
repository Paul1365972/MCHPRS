// Runs the Verilator model of `sim/top.sv` for the host, which talks to it over stdin and stdout:
// - 'W', a length and bytes: the bytes arrive on the serial line, one per byte time.
// - 'R', a clock budget and a capacity: the board clock runs until the board has sent bytes or the
//   budget is spent. The answer is a count and up to `capacity` of the bytes sent.
// While a run is active, the clocks also advance between messages. A paused circuit waits for the
// next message.
#include "Vtop.h"
#include <algorithm>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

static const uint64_t BATCH_BYTES = 256;
static Vtop top;
static uint64_t board_half_period, circuit_half_period, clocks_per_byte;
static uint64_t board_toggle, circuit_toggle, byte_clocks_left;
static bool circuit_clock;
static std::deque<uint8_t> transmitted;

struct Message {
    char kind;
    uint64_t clocks = 0;
    uint32_t capacity = 0;
    std::vector<uint8_t> bytes;
};

static std::deque<Message> messages;
static std::mutex messages_mutex;
static std::condition_variable received_message;
static bool ended = false, input_failed = false;

template <typename T> static bool get(T& value) { return fread(&value, sizeof value, 1, stdin) == 1; }

static void receive() {
    for (char kind; get(kind);) {
        Message message;
        message.kind = kind;
        bool valid = false;
        if (kind == 'W') {
            uint32_t count;
            if (get(count)) {
                message.bytes.resize(count);
                valid = count == 0 || fread(message.bytes.data(), 1, count, stdin) == count;
            }
        } else if (kind == 'R') {
            valid = get(message.clocks) && get(message.capacity);
        }
        if (!valid) {
            input_failed = true;
            break;
        }
        {
            std::lock_guard<std::mutex> lock(messages_mutex);
            messages.push_back(std::move(message));
        }
        received_message.notify_one();
    }
    {
        std::lock_guard<std::mutex> lock(messages_mutex);
        ended = true;
    }
    received_message.notify_one();
}

static void pass_board_clock(bool received_valid, uint8_t received) {
    top.received_valid = received_valid;
    top.received = received;
    top.transmit_ready = byte_clocks_left == 0;
    byte_clocks_left = byte_clocks_left == 0 ? clocks_per_byte - 1 : byte_clocks_left - 1;
    for (bool rose = false; !rose;) {
        uint64_t now = std::min(board_toggle, circuit_toggle);
        if (circuit_toggle == now) {
            circuit_clock = !circuit_clock;
            top.circuit_clock = circuit_clock && top.circuit_clock_enable;
            circuit_toggle += circuit_half_period;
        }
        if (board_toggle == now) {
            rose = !top.board_clock;
            if (rose && top.transmit_valid && top.transmit_ready) transmitted.push_back(top.transmit);
            top.board_clock = rose;
            board_toggle += board_half_period;
        }
        top.eval();
    }
}

static void process(Message message) {
    if (message.kind == 'W') {
        for (uint8_t byte : message.bytes) {
            pass_board_clock(true, byte);
            for (uint64_t gap = 1; gap < clocks_per_byte; gap++) pass_board_clock(false, 0);
        }
    } else {
        uint64_t clocks_left = message.clocks;
        while (transmitted.empty() && clocks_left != 0) {
            uint64_t batch = std::min(clocks_left, BATCH_BYTES * clocks_per_byte);
            for (uint64_t clock = 0; clock < batch; clock++) pass_board_clock(false, 0);
            clocks_left -= batch;
        }
        uint32_t count = std::min<uint64_t>(message.capacity, transmitted.size());
        std::vector<uint8_t> bytes(transmitted.begin(), transmitted.begin() + count);
        transmitted.erase(transmitted.begin(), transmitted.begin() + count);
        fwrite(&count, sizeof count, 1, stdout);
        fwrite(bytes.data(), 1, count, stdout);
        fflush(stdout);
    }
}

int main(int argc, char** argv) {
    if (argc != 4) return 2;
    board_toggle = board_half_period = strtoull(argv[1], nullptr, 10);
    circuit_toggle = circuit_half_period = strtoull(argv[2], nullptr, 10);
    clocks_per_byte = strtoull(argv[3], nullptr, 10);
    if (board_half_period == 0 || circuit_half_period == 0 || clocks_per_byte == 0) return 2;
    top.eval();
    std::thread receiver(receive);
    for (;;) {
        Message message;
        bool have_message = false;
        {
            std::unique_lock<std::mutex> lock(messages_mutex);
            if (messages.empty() && top.circuit_paused && !ended)
                received_message.wait(lock, [] { return ended || !messages.empty(); });
            if (ended && messages.empty()) break;
            if (!messages.empty()) {
                message = std::move(messages.front());
                messages.pop_front();
                have_message = true;
            }
        }
        if (have_message) process(std::move(message));
        else for (uint64_t clock = 0; clock < BATCH_BYTES; clock++) pass_board_clock(false, 0);
    }
    receiver.join();
    return input_failed ? 1 : 0;
}
