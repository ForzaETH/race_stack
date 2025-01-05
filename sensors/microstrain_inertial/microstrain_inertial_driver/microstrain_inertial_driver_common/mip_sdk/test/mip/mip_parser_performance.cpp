
#include "mip/mip.hpp"

#include <algorithm>
#include <vector>
#include <chrono>
#include <numeric>

#include <cstdio>

const uint8_t PING_PACKET[] = {0x75, 0x65, 0x01, 0x02, 0x02, 0x01, 0xE0, 0xC6};

const uint8_t DATA_PACKET[] = {
    0x75,0x65,0x82,0xc1,0x0e,0xd3,0x40,0x8c,0x84,0xef,0x9d,0xb2,0x2d,0x0f,0x00,0x00,
    0x00,0x00,0x0a,0xd5,0x00,0x00,0x00,0xd4,0x7c,0x36,0x4c,0x40,0x10,0x05,0x7f,0xff,
    0xff,0xf8,0x7f,0xc0,0x00,0x00,0x7f,0xff,0xff,0xf8,0x00,0x01,0x10,0x06,0x7f,0xc0,
    0x00,0x00,0x7f,0xc0,0x00,0x00,0x7f,0xc0,0x00,0x00,0x00,0x01,0x10,0x07,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x08,0x10,0x00,0x02,
    0x00,0x00,0x00,0x00,0x1c,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x10,0x02,0x7f,0xc0,0x00,0x00,0x7f,0xc0,0x00,0x00,0x7f,0xc0,0x00,0x00,0x00,0x00,
    0x1c,0x42,0x7f,0xf8,0x00,0x00,0x00,0x00,0x00,0x00,0x7f,0xf8,0x00,0x00,0x00,0x00,
    0x00,0x00,0x7f,0xf8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x09,0x46,0x44,0x64,
    0x26,0x87,0x00,0x04,0x01,0x10,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x10,0x09,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x87,0x56,  // 199 bytes
};

const char NMEA_SENTENCE[] = "$GPGGA,123456.7,1234.5678,N,1234.5678,W,1,5,1.2,12.345,M,12.345,M,*4F";


struct Test
{
    const char* name = nullptr;
    unsigned int num_iterations = 1;
    size_t num_packets = 0;
    std::vector<uint8_t> data;
    std::vector<size_t> chunk_sizes;
};


Test generate_pings()
{
    Test test;

    test.name = "Pings";
    test.num_packets = 200000;
    test.num_iterations = 10;
    test.chunk_sizes = {5, 8, 19};

    test.data.resize(sizeof(PING_PACKET)*test.num_packets);

    for(unsigned int i=0; i<test.num_packets; i++)
        std::copy(std::begin(PING_PACKET), std::end(PING_PACKET), test.data.data()+i*sizeof(PING_PACKET));

    return test;
}

Test generate_long_pkts()
{
    Test test;

    test.name = "Long_Pkts";
    test.num_packets = 100'000;
    test.num_iterations = 10;

    test.data.resize(sizeof(DATA_PACKET)*test.num_packets);

    for(unsigned int i=0; i<test.num_packets; i++)
        std::copy(std::begin(DATA_PACKET),  std::end(DATA_PACKET), test.data.data()+i*sizeof(DATA_PACKET));

    return test;
}

Test generate_interleaved()
{
    Test test;

    test.name = "Interleave_NMEA";
    test.num_packets = 100'000;
    test.num_iterations = 10;

    const size_t INTERVAL = sizeof(DATA_PACKET) + sizeof(NMEA_SENTENCE);
    test.data.resize(INTERVAL*test.num_packets);

    for(unsigned int i=0; i<test.num_packets; i++)
    {
        std::copy(std::begin(DATA_PACKET),   std::end(DATA_PACKET),   test.data.data() + i * INTERVAL);
        std::copy(std::begin(NMEA_SENTENCE), std::end(NMEA_SENTENCE), test.data.data() + i * INTERVAL + sizeof(DATA_PACKET));
    }

    return test;
}

volatile bool dummy = false;
uint8_t parse_buffer[1024];

struct ChunkStats
{
    size_t chunk_size = 0;  // Size of chunks
    size_t num_calls  = 0;  // Calls to parse per test iteration
    float total_time = 0;   // Average total time per test iteration
    float max_time   = 0;   // Max of any single call
    float avg_time   = 0;   // Average of every call
    float med_time   = 0;   // Median of every call
};

//constexpr float kahan_sum(const float* data, size_t count)
//{
//    float result = 0;
//    float c = 0;
//    for(size_t i=0; i<count; i++)
//    {
//        float y = data[i] - c;
//        float t = result + y;
//        c = (t - result) - y;
//        result = t;
//    }
//    return result;
//}


ChunkStats chunked_test(const Test& test, size_t chunk_size)
{
    auto callback = +[](void* v, const mip::PacketRef* p, mip::Timestamp)
    {
        *static_cast<size_t*>(v) += 1;
        return true;
    };
    size_t num_pkts  = 0;
    mip::Parser parser(parse_buffer, sizeof(parse_buffer), callback, &num_pkts, MIPPARSER_DEFAULT_TIMEOUT_MS);


    const size_t num_full_chunks = (chunk_size == 0) ? 1 : (test.data.size() / chunk_size);
    std::vector<float> chunk_times(num_full_chunks * test.num_iterations);

    assert(test.num_iterations > 0);
    for(unsigned int i = 0; i < test.num_iterations; i++)
    {
        parser.reset();
        num_pkts = 0;

        const size_t buffer_size = (chunk_size > 0) ? chunk_size : test.data.size();
        const uint8_t *buffer = test.data.data();

        for(size_t c = 0; c < num_full_chunks; c++)
        {
            auto start = std::chrono::steady_clock::now();
            dummy = true;  // Hopefully prevent compiler/cpu from reordering clock and parse calls.
            parser.parse(buffer, buffer_size, 0);
            dummy = false;
            auto stop = std::chrono::steady_clock::now();

            std::chrono::duration<float> duration = stop - start;

            chunk_times[i * num_full_chunks + c] = duration.count();
            buffer += chunk_size;
        }
        if(chunk_size > 0)
            parser.parse(test.data.data() + num_full_chunks * chunk_size, test.data.size() - num_full_chunks * chunk_size, 0);

        if(num_pkts != test.num_packets)
            fprintf(stderr, "Error: Got %zu packets but expected %zu!\n", num_pkts, test.num_packets);
    }

    ChunkStats stats;
    stats.chunk_size = chunk_size;
    stats.num_calls  = num_full_chunks;
    stats.total_time = (float)std::accumulate(chunk_times.begin(), chunk_times.end(), 0.0) / test.num_iterations;  // Accumulate with double precision!
    stats.avg_time   = stats.total_time / num_full_chunks;
    stats.max_time   = *std::max_element(chunk_times.begin(), chunk_times.end());

    std::sort(chunk_times.begin(), chunk_times.end());
    stats.med_time   = chunk_times[chunk_times.size()/2];

    return stats;
}


int main(int argc, const char* argv[])
{
    std::initializer_list<Test> tests = {
        generate_pings(),
        generate_long_pkts(),
        generate_interleaved()
    };

    std::initializer_list<size_t> chunk_sizes = {5, 8, 19, 53, 127, 512, 1024, 8192};

    std::vector<ChunkStats> stats(tests.size() * (chunk_sizes.size()+1));

    size_t t = 0;
    for(const Test& test : tests)
    {
        printf("Test %s...\n", test.name);

        size_t s = 0;
        for(size_t chunk_size : chunk_sizes)
        {
            printf("  Chunk size %zu\n", chunk_size);
            stats[t * (chunk_sizes.size()+1) + s] = chunked_test(test, chunk_size);
            s++;
        }
        printf("  Chunk size oo\n");
        stats[t * (chunk_sizes.size()+1) + s] = chunked_test(test, test.data.size());

        t++;
    }

    std::printf("\n\n"
        "       Test          Total     Chunk      Parse     Mean       Median     Max        Though     Through  \n"
        "       Name          Bytes     Size[B]    Calls     Time[us]   Time[us]   Time[us]    [MB/s]    [MPkt/s] \n"
        "---------------------------------------------------------------------------------------------------------\n"
    );

    t=0;
    for(const Test& test : tests)
    {
        size_t s=0;
        for(size_t chunk_size : chunk_sizes)
        {
            const ChunkStats& stat = stats[t * (chunk_sizes.size()+1) + s];

            std::printf("%16s   %8zu   %8zu   %8zu   %8.2f   %8.2f   %8.2f   %8.2f   %8.2f\n",
                test.name, stat.chunk_size * stat.num_calls, chunk_size, stat.num_calls,
                stat.avg_time*1e6f, stat.med_time*1e6f, stat.max_time*1e6f,
                test.data.size() / stat.total_time / 1e6f,
                test.num_packets / stat.total_time / 1e6f
            );

            s++;
        }
        const ChunkStats& stat = stats[t * (chunk_sizes.size()+1) + s];

        std::printf("%16s   %8zu   infinite   %8zu   %8.2f   %8.2f   %8.2f   %8.2f   %8.2f\n",
                test.name, stat.chunk_size * stat.num_calls, stat.num_calls,
                stat.avg_time*1e6f, stat.med_time*1e6f, stat.max_time*1e6f,
                test.data.size() / stat.total_time / 1e6f,
                test.num_packets / stat.total_time / 1e6f
        );

        t++;
    }

    return 0;
}
