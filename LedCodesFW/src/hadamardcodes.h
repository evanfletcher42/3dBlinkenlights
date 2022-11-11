#pragma once

#include <stdint.h>

namespace Hadamard
{
    // (8, 4, 2)_2 augmented Hadamard code, with alphabet 0->01, 1->10 to ensure edges always exist
    const uint8_t H[ 8*4*2 ] = 
    {
        0,1, 0,1, 0,1, 0,1,
        0,1, 1,0, 0,1, 1,0,
        0,1, 0,1, 1,0, 1,0,
        0,1, 1,0, 1,0, 0,1,
        1,0, 1,0, 1,0, 1,0,
        1,0, 0,1, 1,0, 0,1,
        1,0, 1,0, 0,1, 0,1,
        1,0, 0,1, 0,1, 1,0
    };

    const uint8_t* pEncode( uint8_t x )
    {
        return &( H[ x * 4 * 2] );
    }

    const void pEncodeRGB( uint16_t x, const uint8_t* &pEncodedR, const uint8_t* &pEncodedG, const uint8_t* &pEncodedB)
    {
        pEncodedR = pEncode( ( uint8_t )( ( x >> 0 ) & 0b111 ) );
        pEncodedG = pEncode( ( uint8_t )( ( x >> 3 ) & 0b111 ) );
        pEncodedB = pEncode( ( uint8_t )( ( x >> 6 ) & 0b111 ) );
    }
}
