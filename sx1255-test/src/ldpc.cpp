#include "ldpc.h"
#include <string.h>

static uint32_t G_CIRCULANTS[28][16] = {
    {0x19B57ED5, 0xBFDE0A55, 0x9898ABFB, 0xEBC3907F, 0x8477304D, 0x9052388C, 0x15257358, 0x023CE4B2, 0xFB1DA58B, 0xEA20FA59, 0x9B4DEB6B, 0x6497B728, 0x51ECBAAA, 0xAA2825AE, 0x316EB09C, 0x1CD7613C},
    {0xF7C52AF4, 0xD57AD734, 0x0F48BAF9, 0xAF1DE21B, 0x3401856E, 0x3DFBB18F, 0xD1F1C855, 0xE37E9FAC, 0xF538B461, 0xDBC64197, 0x37F441C9, 0xFDBB23FD, 0xDBBBF386, 0xA0CF1541, 0xCE747EC1, 0x75A2F2DA},
    {0x7863D95C, 0x036CFC45, 0x47A090CD, 0x63E3AACD, 0x88D66B3A, 0x0607C3FA, 0x10CF2F67, 0x3E30C863, 0x31B08617, 0xFAAF07BC, 0x6D76F3CC, 0x2434357C, 0x24315209, 0x4BE73D15, 0x5575C767, 0x43954226},
    {0xF683896C, 0xADB8AA54, 0xDBAEE7CC, 0x835D90ED, 0x6CD10BFF, 0x6BBBE4E3, 0x7897F907, 0x384F5F7F, 0x768FB0B5, 0x1B2D4B5A, 0x4E5A06D0, 0xA888550A, 0x9938E787, 0xFE92C453, 0x18A280E7, 0x7B8FF855},
    {0xBAC753F4, 0xB5F217DE, 0x6C1BC646, 0xF7F3CA3B, 0x9F511001, 0x98502927, 0x08F37DF9, 0x95B03FF2, 0xE0BB8FC2, 0x84828C0A, 0xC1918F27, 0x543D87DA, 0xBCBE70A0, 0xBBF5DCE5, 0x0AA349D3, 0x535B287B},
    {0x75DC7D90, 0xC1821C7F, 0xDF58756F, 0xBCC242F5, 0xA692B9BA, 0x5320A770, 0x473FF86C, 0x328E67DE, 0x381EE326, 0xA3F0A926, 0x9935AC1D, 0xD2AA847C, 0xD04B5BD0, 0xF343BB14, 0xA9079E72, 0x0C24FAA5},
    {0xC403AB29, 0xDFB67D84, 0xF76B64B5, 0x4C2E99FF, 0xAAE4DAAE, 0x6AEF9DB3, 0xFD714AAE, 0xECE27E06, 0x6B25F4A5, 0x3A96B2E9, 0xB8209F8B, 0xD90BF292, 0x9D7A4B84, 0xF1B7E5F9, 0x85DF5BE9, 0x469D95A4},
    {0x80863F48, 0x5C7F1F0D, 0xD95CDB87, 0xE9817E85, 0x9DB1012F, 0xA69D753B, 0xECF7E6F7, 0x39E888EC, 0x45A22012, 0xE8CC5D5F, 0x19ACA515, 0x421593A9, 0x834089FC, 0x7D621CC4, 0x0B023879, 0x7262A18C},
    {0x3AF5408C, 0x917060C1, 0x32DA83AE, 0xF6883D4A, 0x3C08112C, 0xEECB7C44, 0xF05C60F3, 0x2377EBBE, 0x77D303A6, 0x8C2795A5, 0xED1B6CE5, 0x8ED3A827, 0x7F2A747F, 0xEB380B7C, 0x11E1AAD7, 0x21B67616},
    {0x003280DD, 0x1D6D741F, 0x11288DD8, 0xA1A38112, 0xEB642B1C, 0x2D4D839E, 0x896E0A8D, 0x97173829, 0xA49B714B, 0xE9345E63, 0xC32E14FF, 0xFDC60439, 0x4686C117, 0xC96C8257, 0x4F1794E0, 0x3A4E1131},
    {0xE383A67C, 0xDBCBD5F5, 0xE0E740AC, 0x6599FDEB, 0xA8FE2EDD, 0x66E8F084, 0x1E4CB0FF, 0x71ECD9BC, 0x665A170F, 0xBEF719F6, 0x2F8DBCD2, 0xA995B395, 0xB278445B, 0x890876DA, 0xBE8792C1, 0x69248815},
    {0xEF4B6961, 0x93E7C98C, 0xED89A050, 0x0285C48D, 0x1434CB0E, 0xC47BEBDB, 0x60660E95, 0xA1DBD03E, 0xD27880F8, 0x6414678B, 0x76D7333D, 0xF7B36883, 0x8624E67F, 0x0064A120, 0x6E91DDD2, 0x3B6F59D3},
    {0xDBAAF4C8, 0x6FF62062, 0x9C127052, 0xCF4F47CC, 0xA479DFDE, 0xB8236FC6, 0x0A9C5F23, 0x4ADC06AE, 0xF4A83FD4, 0xE6EAAB23, 0x0959C3D0, 0x64C7579E, 0xC34F7A28, 0x4C2006A8, 0x10A4D045, 0x76B93874},
    {0x9A28B5DD, 0xC00F6D2D, 0xEBA4D7CA, 0x270ABB9C, 0x5A3FAAE3, 0x903D7E17, 0xBD213676, 0x9BB4F418, 0xFAB551D1, 0x83455EE4, 0xBCAF44CE, 0x88702E03, 0x71CB1EC6, 0x7F85A4CA, 0x50F35F4E, 0x60F7724B},
    {0xB8CFD694, 0xB89FD558, 0xE513B96C, 0x503E34B1, 0xF50AF142, 0x73083C1A, 0x8862CCDD, 0x1AFCB273, 0x46F58035, 0xC698BDD5, 0x17B0F1AE, 0x6256B16D, 0x3DA4B670, 0x047DFD17, 0x8E2DD927, 0x1C6E485F},
    {0xEFFAC76C, 0x8F9EF440, 0xCAA005D6, 0x5492BBFC, 0x6CD601E0, 0x3FE34B50, 0x38144725, 0xF8FB7B23, 0x66CA7CB9, 0xA9CFF107, 0x3739EBC8, 0x7231D010, 0xD304A67F, 0x6E6AC27D, 0x8BE6ACF7, 0x00155489},
    {0x8B968800, 0xB3F7D3CC, 0xCB39DEA4, 0x4CDAFE39, 0xBAF9E0C7, 0x185D0F26, 0x453A9CDE, 0x5E95B460, 0x9B02A2D3, 0x19397F2E, 0xACE080A9, 0x624040A9, 0xABB89361, 0x3122EED8, 0xD4438B29, 0x16017E6A},
    {0x80B3FF6D, 0xA12B8428, 0x6CDB87BF, 0x20F824C1, 0x41D08620, 0x7C1EEFB0, 0x42F356B5, 0xEC207521, 0x9832DFD1, 0xF886D02E, 0x06E1DB5A, 0x08D4A041, 0x83D99F19, 0x35F16353, 0x9DDB3DC9, 0x3D4986BB},
    {0xE66A6CBD, 0x3385A0FB, 0x5B2222A9, 0x459DA82B, 0xBA64D5CF, 0x6C0179C3, 0x85C91366, 0x9575777F, 0x70533529, 0xE12C33B4, 0x0CD5E34F, 0xF407F1D3, 0x45060E57, 0x0C933061, 0x1A72A9A2, 0x26BB7A54},
    {0x67EBF265, 0xA7C209D7, 0x832DBCDF, 0xDED5D51C, 0x403F0757, 0xD123CC96, 0x64825CE2, 0x3CD690BF, 0x4F83E15A, 0x7A027AD7, 0xA13C0754, 0xCFBFC8F1, 0x6D48DDC5, 0x8D33D0F3, 0x6AE7CE97, 0x6F3C2BD8},
    {0x927FC7B0, 0xABA7A4C4, 0x38896307, 0x4CBBBF32, 0x4C837739, 0xFFB2F91D, 0x0C121382, 0x36D7F2D2, 0xB46F971D, 0xEC48991A, 0xAE658801, 0x2574FF95, 0x145ED3BE, 0x03B44B24, 0x8FA7FA05, 0x21FF0293},
    {0x7EF0F5AC, 0x0D9EDFFA, 0xBBC63E87, 0x874F3A4A, 0x3B05FC6D, 0x54F73D1A, 0x8C64FB46, 0x7E95CCBE, 0xB1020469, 0x253E4887, 0x7DB26F72, 0xDE997505, 0x8D6F1440, 0xFC5FD032, 0x81E8A9CF, 0x785EB17A},
    {0x2A2DC841, 0x7C6C1392, 0xA2513576, 0xF695C0CC, 0x42B6188B, 0x4B936ADA, 0x1199371C, 0x88C1B99E, 0xD50D189B, 0xE8897F89, 0x0360044E, 0xEDE17BEA, 0x0423291B, 0x9A2B0489, 0x415EDD1D, 0x0EDC6160},
    {0xE11C2F3D, 0x3AD148F8, 0x33394A55, 0xA617CE76, 0x12EDF478, 0x9B8DB68F, 0xAA2724C4, 0x1B4F820D, 0x763D7BE2, 0xB41293AC, 0x071A8801, 0xA99971BD, 0xAB9E8718, 0xE70153C9, 0xB2FE8118, 0x6C15DC51},
    {0x88AEF75D, 0x2FADCDD2, 0xFC7758E9, 0xD691D92C, 0x210570A4, 0x194A9D92, 0x845B4E5B, 0x69529D4A, 0x20C88DD5, 0xE72F858C, 0x60AEEF6C, 0xA266752B, 0x152DD847, 0x0AC894AD, 0x0EB47809, 0x58B00BE2},
    {0x084E79CD, 0x67AC70F3, 0x1A9A1528, 0x417F244D, 0xBB4B8E65, 0x9E8AD430, 0x13BEEEC2, 0x9BCCAA0E, 0xF9F35024, 0x21EE9833, 0xB7CE6C74, 0x39C6E17E, 0x6BAA5462, 0x2B052A8A, 0xA2F2DDCD, 0x5B4D8AB4},
    {0xE9F418D8, 0x0B7C73B5, 0x40283E39, 0x57ABD598, 0xF3BBC284, 0xA1295F0E, 0x38FF23D2, 0x7970AEB4, 0x10A5D8E5, 0x70E74368, 0x9EB5BB1A, 0x10D10038, 0x67015077, 0xB15CD1FE, 0xCA7CC67E, 0x49090234},
    {0x0242BF08, 0x70664F21, 0x5E0A6D10, 0xDE4BD067, 0x05271778, 0xD25F7396, 0xAE107FE5, 0x6D68FD1D, 0xA876DB0C, 0x0B40A8B4, 0xD756FBD6, 0xBF3ABA54, 0x87DB7392, 0x4D7487D5, 0xF4AA2C38, 0x3E755200},
};

__attribute__((always_inline)) __attribute__((section(".time_critical.shift_left_511"))) static inline void shift_left_511(uint32_t *word) {
#if defined(__ARM_ARCH_8M_MAIN__)
    uint32_t *p = word;
    __asm__ volatile("ldr r1, [%[ptr], #60] \n\t" // Load word[15]
                     "lsrs r1, r1, #31 \n\t"      // Shift Right by 31. Carry Flag = bit 510!
                     "ldmia %[ptr], {r1, r2, r3, r4, r5, r6, r8, r9} \n\t" // Load first 8 words (no r7)
                     "adcs r1, r1 \n\t"           // word[0] = (word[0] << 1) | Carry
                     "adcs r2, r2 \n\t"
                     "adcs r3, r3 \n\t"
                     "adcs r4, r4 \n\t"
                     "adcs r5, r5 \n\t"
                     "adcs r6, r6 \n\t"
                     "adcs r8, r8 \n\t"
                     "adcs r9, r9 \n\t"
                     "stmia %[ptr]!, {r1, r2, r3, r4, r5, r6, r8, r9} \n\t" // Store and increment ptr
                     "ldmia %[ptr], {r1, r2, r3, r4, r5, r6, r8, r9} \n\t"  // Load next 8 words
                     "adcs r1, r1 \n\t"
                     "adcs r2, r2 \n\t"
                     "adcs r3, r3 \n\t"
                     "adcs r4, r4 \n\t"
                     "adcs r5, r5 \n\t"
                     "adcs r6, r6 \n\t"
                     "adcs r8, r8 \n\t"
                     "adcs r9, r9 \n\t"
                     "bic r9, r9, #0x80000000 \n\t" // Mask out bit 511 (highest bit of word 15, which is r9 here)
                     "stmia %[ptr], {r1, r2, r3, r4, r5, r6, r8, r9} \n\t"   // Store final 8 words
                     : [ptr] "+r"(p)
                     :
                     : "r1", "r2", "r3", "r4", "r5", "r6", "r8", "r9", "memory", "cc");
#else
    uint32_t bit_510 = (word[15] >> 30) & 1; // Bit 510 wraps around to Bit 0
    word[15] = ((word[15] << 1) | (word[14] >> 31)) & 0x7FFFFFFF;
    word[14] = (word[14] << 1) | (word[13] >> 31);
    word[13] = (word[13] << 1) | (word[12] >> 31);
    word[12] = (word[12] << 1) | (word[11] >> 31);
    word[11] = (word[11] << 1) | (word[10] >> 31);
    word[10] = (word[10] << 1) | (word[9] >> 31);
    word[9] = (word[9] << 1) | (word[8] >> 31);
    word[8] = (word[8] << 1) | (word[7] >> 31);
    word[7] = (word[7] << 1) | (word[6] >> 31);
    word[6] = (word[6] << 1) | (word[5] >> 31);
    word[5] = (word[5] << 1) | (word[4] >> 31);
    word[4] = (word[4] << 1) | (word[3] >> 31);
    word[3] = (word[3] << 1) | (word[2] >> 31);
    word[2] = (word[2] << 1) | (word[1] >> 31);
    word[1] = (word[1] << 1) | (word[0] >> 31);
    word[0] = (word[0] << 1) | bit_510;
#endif
}

__attribute__((always_inline)) static inline uint32_t reverse_bits_32(uint32_t x) {
#if defined(__ARM_ARCH) && __ARM_ARCH >= 7
    return __builtin_arm_rbit(x);
#else
    // Software fallback for ARMv6-M (Cortex-M0+) or non-ARM (e.g. host simulations)
    x = ((x >> 1) & 0x55555555) | ((x & 0x55555555) << 1);
    x = ((x >> 2) & 0x33333333) | ((x & 0x33333333) << 2);
    x = ((x >> 4) & 0x0F0F0F0F) | ((x & 0x0F0F0F0F) << 4);
    x = ((x >> 8) & 0x00FF00FF) | ((x & 0x00FF00FF) << 8);
    return (x >> 16) | (x << 16);
#endif
}

__attribute__((section(".time_critical.ldpc_78_encode"))) void ldpc_78_encode(const uint8_t *info_bytes, uint16_t info_len, uint8_t *codeword_out) {
    // 1. Clear the codeword buffer
    memset(codeword_out, 0, 1020);

    // 2. Copy the byte-aligned information data into the codeword buffer.
    // The (8160, 7136) info block is 7136 bits (892 bytes).
    uint16_t safe_len = info_len > 892 ? 892 : info_len;
    memcpy(codeword_out, info_bytes, safe_len);

    uint32_t parity0[16] = {0};
    uint32_t parity1[16] = {0};

    int word_idx = 0;
    uint32_t current_word;
    // Safely load 4 bytes into a 32-bit integer regardless of pointer alignment or strict aliasing rules
    memcpy(&current_word, &codeword_out[0], sizeof(uint32_t));
    current_word = __builtin_bswap32(current_word);
    uint32_t bit_mask = 0x80000000;

    // 2. Parity Accumulation
    for (int info_blk = 0; info_blk < 14; info_blk++) {
        uint32_t circ0[16];
        uint32_t circ1[16];
        memcpy(circ0, G_CIRCULANTS[info_blk * 2 + 0], 64);
        memcpy(circ1, G_CIRCULANTS[info_blk * 2 + 1], 64);

        int start_bit = 0;
        if (info_blk == 0) {
            for (int i = 0; i < 18; i++) {
                shift_left_511(circ0);
                shift_left_511(circ1);
            }
            start_bit = 18;
        }

        for (int bit_idx = start_bit; bit_idx < 511; bit_idx++) {
            uint32_t m_k = current_word & bit_mask;
            bit_mask >>= 1;
            if (!bit_mask) {
                bit_mask = 0x80000000;
                word_idx++;
                memcpy(&current_word, &codeword_out[word_idx * 4], sizeof(uint32_t));
                current_word = __builtin_bswap32(current_word);
            }

            uint32_t mask = -(m_k != 0); // Creates 0x00000000 if m_k is 0, or 0xFFFFFFFF if m_k is non-zero.

            // Loop unrolled XOR operations
            parity0[0] ^= (circ0[0] & mask);
            parity1[0] ^= (circ1[0] & mask);
            parity0[1] ^= (circ0[1] & mask);
            parity1[1] ^= (circ1[1] & mask);
            parity0[2] ^= (circ0[2] & mask);
            parity1[2] ^= (circ1[2] & mask);
            parity0[3] ^= (circ0[3] & mask);
            parity1[3] ^= (circ1[3] & mask);
            parity0[4] ^= (circ0[4] & mask);
            parity1[4] ^= (circ1[4] & mask);
            parity0[5] ^= (circ0[5] & mask);
            parity1[5] ^= (circ1[5] & mask);
            parity0[6] ^= (circ0[6] & mask);
            parity1[6] ^= (circ1[6] & mask);
            parity0[7] ^= (circ0[7] & mask);
            parity1[7] ^= (circ1[7] & mask);
            parity0[8] ^= (circ0[8] & mask);
            parity1[8] ^= (circ1[8] & mask);
            parity0[9] ^= (circ0[9] & mask);
            parity1[9] ^= (circ1[9] & mask);
            parity0[10] ^= (circ0[10] & mask);
            parity1[10] ^= (circ1[10] & mask);
            parity0[11] ^= (circ0[11] & mask);
            parity1[11] ^= (circ1[11] & mask);
            parity0[12] ^= (circ0[12] & mask);
            parity1[12] ^= (circ1[12] & mask);
            parity0[13] ^= (circ0[13] & mask);
            parity1[13] ^= (circ1[13] & mask);
            parity0[14] ^= (circ0[14] & mask);
            parity1[14] ^= (circ1[14] & mask);
            parity0[15] ^= (circ0[15] & mask);
            parity1[15] ^= (circ1[15] & mask);

            shift_left_511(circ0);
            shift_left_511(circ1);
        }
    }

    // 3. Pack parity bits into the final codeword output
    // Parity block begins exactly at bit 7136 (byte 892).
    // Per CCSDS 131.0-B-5: parity block is 1022 bits (511 + 511) + 2 appended zero bits = 1024 bits total (128 bytes).
    // Concatenate the two 511-bit blocks into a continuous 32-word (1024-bit) array in LSB-first order.
    uint32_t unified_parity[32];
    for (int i = 0; i < 15; i++) {
        unified_parity[i] = parity0[i];
    }
    // Blend the 511th bit of parity0 (bit 30 of parity0[15]) with bit 0 of parity1[0] (placed at bit 31 of unified_parity[15])
    unified_parity[15] = (parity0[15] & 0x7FFFFFFF) | (parity1[0] << 31);
    // Align and copy the rest of parity1 (shifted by 1 bit)
    for (int i = 16; i < 31; i++) {
        unified_parity[i] = (parity1[i - 16] >> 1) | (parity1[i - 15] << 31);
    }
    unified_parity[31] = (parity1[15] >> 1) & 0x3FFFFFFF; // Clear bits 30 & 31 of final word (trailing 2 zero bits)

    // Write the packed parity words directly to codeword_out using hardware bit reversal and byte reversing stores
    uint32_t *dst = (uint32_t *)(codeword_out + 892);
    for (int i = 0; i < 32; i++) {
        dst[i] = __builtin_bswap32(reverse_bits_32(unified_parity[i]));
    }
}