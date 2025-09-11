#include <stdio.h>
#include <stdint.h>
#include <stddef.h>

void vulnerable_copy(const uint8_t *data, size_t size) {
    char buffer[16];
    for (size_t i = 0; i < size; i++) {
        buffer[i] = data[i];  // no boundary check -> easy to crash
    }
}

int main(int argc, char **argv) {
    if (argc != 2) return 0;
    FILE *f = fopen(argv[1], "rb");
    if (!f) return 0;
    uint8_t buf[1024];
    size_t n = fread(buf, 1, sizeof(buf), f);
    fclose(f);
    vulnerable_copy(buf, n);
    return 0;
}