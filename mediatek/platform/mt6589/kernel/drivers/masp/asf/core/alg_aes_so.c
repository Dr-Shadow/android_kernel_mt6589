/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 *
 * MediaTek Inc. (C) 2011. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

#include "sec_osal_light.h"
#include "sec_log.h"
#include "aes_so.h"

/**************************************************************************
 *  TYPEDEF
 **************************************************************************/
typedef unsigned int uint32;
typedef unsigned char uchar;

/**************************************************************************
 *  DEFINITIONS
 **************************************************************************/
#define MOD                             "AES_SO"
#define CIPHER_BLOCK_SIZE               (16)

#define CT_AES128_LEN                   16      // 16B (AES128)
#define CT_AES192_LEN                   24      // 24B (AES192)
#define CT_AES256_LEN                   32      // 32B (AES256)

/**************************************************************************
 *  EXTERNAL FUNCTION
 **************************************************************************/
extern void * mcpy(void *dest, const void *src, int  cnt);

/**************************************************************************
 *  GLOBAL VARIABLES
 **************************************************************************/
uint32                                  aes_key_len = 0;

static uchar FSb[256];
static ulong FT0[256]; 
static ulong FT1[256]; 
static ulong FT2[256]; 
static ulong FT3[256]; 

static uchar RSb[256];
static ulong RT0[256];
static ulong RT1[256];
static ulong RT2[256];
static ulong RT3[256];

static ulong RCON[10];

static int aes_init_done = 0;

static int pow[256];
static int log[256];


/**************************************************************************
 *  MTK SECRET
 **************************************************************************/
static uint32 g_AES_IV[4]= {
    0x6c8d3259, 0x86911412, 0x55975412, 0x6c8d3257
};

static uint32 g_AES_IV_TEMP[4]= {
    0x0,0x0,0x0,0x0
};

uint32 g_AES_Key[4] = {
    0x0, 0x0, 0x0, 0x0
};

a_ctx aes;

#ifndef G_U_LE
#define G_U_LE(n,b,i)                           \
{                                               \
    (n) = ( (ulong) (b)[(i)    ]       )        \
        | ( (ulong) (b)[(i) + 1] <<  8 )        \
        | ( (ulong) (b)[(i) + 2] << 16 )        \
        | ( (ulong) (b)[(i) + 3] << 24 );       \
}
#endif

#ifndef P_U_LE
#define P_U_LE(n,b,i)                           \
{                                               \
    (b)[(i)    ] = (uchar) ( (n)       );       \
    (b)[(i) + 1] = (uchar) ( (n) >>  8 );       \
    (b)[(i) + 2] = (uchar) ( (n) >> 16 );       \
    (b)[(i) + 3] = (uchar) ( (n) >> 24 );       \
}
#endif

#define ROTL8(x) ( ( x << 8 ) & 0xFFFFFFFF ) | ( x >> 24 )
#define XTIME(x) ( ( x << 1 ) ^ ( ( x & 0x80 ) ? 0x1B : 0x00 ) )
#define MUL(x,y) ( ( x && y ) ? pow[(log[x]+log[y]) % 255] : 0 )

/**************************************************************************
 *  FUNCTIONS
 **************************************************************************/
static void a_gen_tables( void )
{
    int i, x, y, z;

    for( i = 0, x = 1; i < 256; i++ )
    {
        pow[i] = x;
        log[x] = i;
        x = ( x ^ XTIME( x ) ) & 0xFF;
    }

    for( i = 0, x = 1; i < 10; i++ )
    {
        RCON[i] = (ulong) x;
        x = XTIME( x ) & 0xFF;
    }

    FSb[0x00] = 0x63;
    RSb[0x63] = 0x00;

    for( i = 1; i < 256; i++ )
    {
        x = pow[255 - log[i]];

        y  = x; y = ( (y << 1) | (y >> 7) ) & 0xFF;
        x ^= y; y = ( (y << 1) | (y >> 7) ) & 0xFF;
        x ^= y; y = ( (y << 1) | (y >> 7) ) & 0xFF;
        x ^= y; y = ( (y << 1) | (y >> 7) ) & 0xFF;
        x ^= y ^ 0x63;

        FSb[i] = (uchar) x;
        RSb[x] = (uchar) i;
    }

    for( i = 0; i < 256; i++ )
    {
        x = FSb[i];
        y = XTIME( x ) & 0xFF;
        z =  ( y ^ x ) & 0xFF;

        FT0[i] = ( (ulong) y       ) ^
                     ( (ulong) x <<  8 ) ^
                     ( (ulong) x << 16 ) ^
                     ( (ulong) z << 24 );

        FT1[i] = ROTL8( FT0[i] );
        FT2[i] = ROTL8( FT1[i] );
        FT3[i] = ROTL8( FT2[i] );

        x = RSb[i];

        RT0[i] = ( (ulong) MUL( 0x0E, x )       ) ^
                     ( (ulong) MUL( 0x09, x ) <<  8 ) ^
                     ( (ulong) MUL( 0x0D, x ) << 16 ) ^
                     ( (ulong) MUL( 0x0B, x ) << 24 );

        RT1[i] = ROTL8( RT0[i] );
        RT2[i] = ROTL8( RT1[i] );
        RT3[i] = ROTL8( RT2[i] );
    }
}


int a_enc (a_ctx *ctx, const uchar *key, uint32 keysize)
{
    uint32 i;
    ulong *RK;

    if( aes_init_done == 0 )
    {
        a_gen_tables();
        aes_init_done = 1;
    }

    switch( keysize )
    {
        case 128: ctx->nr = 10; break;
        case 192: ctx->nr = 12; break;
        case 256: ctx->nr = 14; break;
        default : return( E_AES_INVALID_KEY_LENGTH );
    }

    ctx->rk = RK = ctx->buf;

    for( i = 0; i < (keysize >> 5); i++ )
    {
        G_U_LE( RK[i], key, i << 2 );
    }

    switch( ctx->nr )
    {
        case 10:

            for( i = 0; i < 10; i++, RK += 4 )
            {
                RK[4]  = RK[0] ^ RCON[i] ^
                ( (ulong) FSb[ ( RK[3] >>  8 ) & 0xFF ]       ) ^
                ( (ulong) FSb[ ( RK[3] >> 16 ) & 0xFF ] <<  8 ) ^
                ( (ulong) FSb[ ( RK[3] >> 24 ) & 0xFF ] << 16 ) ^
                ( (ulong) FSb[ ( RK[3]       ) & 0xFF ] << 24 );

                RK[5]  = RK[1] ^ RK[4];
                RK[6]  = RK[2] ^ RK[5];
                RK[7]  = RK[3] ^ RK[6];
            }
            break;

        case 12:

            for( i = 0; i < 8; i++, RK += 6 )
            {
                RK[6]  = RK[0] ^ RCON[i] ^
                ( (ulong) FSb[ ( RK[5] >>  8 ) & 0xFF ]       ) ^
                ( (ulong) FSb[ ( RK[5] >> 16 ) & 0xFF ] <<  8 ) ^
                ( (ulong) FSb[ ( RK[5] >> 24 ) & 0xFF ] << 16 ) ^
                ( (ulong) FSb[ ( RK[5]       ) & 0xFF ] << 24 );

                RK[7]  = RK[1] ^ RK[6];
                RK[8]  = RK[2] ^ RK[7];
                RK[9]  = RK[3] ^ RK[8];
                RK[10] = RK[4] ^ RK[9];
                RK[11] = RK[5] ^ RK[10];
            }
            break;

        case 14:

            for( i = 0; i < 7; i++, RK += 8 )
            {
                RK[8]  = RK[0] ^ RCON[i] ^
                ( (ulong) FSb[ ( RK[7] >>  8 ) & 0xFF ]       ) ^
                ( (ulong) FSb[ ( RK[7] >> 16 ) & 0xFF ] <<  8 ) ^
                ( (ulong) FSb[ ( RK[7] >> 24 ) & 0xFF ] << 16 ) ^
                ( (ulong) FSb[ ( RK[7]       ) & 0xFF ] << 24 );

                RK[9]  = RK[1] ^ RK[8];
                RK[10] = RK[2] ^ RK[9];
                RK[11] = RK[3] ^ RK[10];

                RK[12] = RK[4] ^
                ( (ulong) FSb[ ( RK[11]       ) & 0xFF ]       ) ^
                ( (ulong) FSb[ ( RK[11] >>  8 ) & 0xFF ] <<  8 ) ^
                ( (ulong) FSb[ ( RK[11] >> 16 ) & 0xFF ] << 16 ) ^
                ( (ulong) FSb[ ( RK[11] >> 24 ) & 0xFF ] << 24 );

                RK[13] = RK[5] ^ RK[12];
                RK[14] = RK[6] ^ RK[13];
                RK[15] = RK[7] ^ RK[14];
            }
            break;

        default:

            break;
    }

    return( 0 );
}

int a_dec (a_ctx *ctx, const uchar *key, uint32 keysize)
{
    int i, j;
    a_ctx cty;
    ulong *RK;
    ulong *SK;
    int ret;

    switch( keysize )
    {
        case 128: ctx->nr = 10; break;
        case 192: ctx->nr = 12; break;
        case 256: ctx->nr = 14; break;
        default : return( E_AES_INVALID_KEY_LENGTH );
    }

    ctx->rk = RK = ctx->buf;

    ret = a_enc( &cty, key, keysize );
    if( ret != 0 )
        return( ret );

    SK = cty.rk + cty.nr * 4;

    *RK++ = *SK++;
    *RK++ = *SK++;
    *RK++ = *SK++;
    *RK++ = *SK++;

    for( i = ctx->nr - 1, SK -= 8; i > 0; i--, SK -= 8 )
    {
        for( j = 0; j < 4; j++, SK++ )
        {
            *RK++ = RT0[ FSb[ ( *SK       ) & 0xFF ] ] ^
                    RT1[ FSb[ ( *SK >>  8 ) & 0xFF ] ] ^
                    RT2[ FSb[ ( *SK >> 16 ) & 0xFF ] ] ^
                    RT3[ FSb[ ( *SK >> 24 ) & 0xFF ] ];
        }
    }

    *RK++ = *SK++;
    *RK++ = *SK++;
    *RK++ = *SK++;
    *RK++ = *SK++;

    memset( &cty, 0, sizeof( a_ctx ) );

    return( 0 );
}

#define A_F(X0,X1,X2,X3,Y0,Y1,Y2,Y3)     \
{                                               \
    X0 = *RK++ ^ FT0[ ( Y0       ) & 0xFF ] ^   \
                 FT1[ ( Y1 >>  8 ) & 0xFF ] ^   \
                 FT2[ ( Y2 >> 16 ) & 0xFF ] ^   \
                 FT3[ ( Y3 >> 24 ) & 0xFF ];    \
                                                \
    X1 = *RK++ ^ FT0[ ( Y1       ) & 0xFF ] ^   \
                 FT1[ ( Y2 >>  8 ) & 0xFF ] ^   \
                 FT2[ ( Y3 >> 16 ) & 0xFF ] ^   \
                 FT3[ ( Y0 >> 24 ) & 0xFF ];    \
                                                \
    X2 = *RK++ ^ FT0[ ( Y2       ) & 0xFF ] ^   \
                 FT1[ ( Y3 >>  8 ) & 0xFF ] ^   \
                 FT2[ ( Y0 >> 16 ) & 0xFF ] ^   \
                 FT3[ ( Y1 >> 24 ) & 0xFF ];    \
                                                \
    X3 = *RK++ ^ FT0[ ( Y3       ) & 0xFF ] ^   \
                 FT1[ ( Y0 >>  8 ) & 0xFF ] ^   \
                 FT2[ ( Y1 >> 16 ) & 0xFF ] ^   \
                 FT3[ ( Y2 >> 24 ) & 0xFF ];    \
}

#define A_R(X0,X1,X2,X3,Y0,Y1,Y2,Y3)     \
{                                               \
    X0 = *RK++ ^ RT0[ ( Y0       ) & 0xFF ] ^   \
                 RT1[ ( Y3 >>  8 ) & 0xFF ] ^   \
                 RT2[ ( Y2 >> 16 ) & 0xFF ] ^   \
                 RT3[ ( Y1 >> 24 ) & 0xFF ];    \
                                                \
    X1 = *RK++ ^ RT0[ ( Y1       ) & 0xFF ] ^   \
                 RT1[ ( Y0 >>  8 ) & 0xFF ] ^   \
                 RT2[ ( Y3 >> 16 ) & 0xFF ] ^   \
                 RT3[ ( Y2 >> 24 ) & 0xFF ];    \
                                                \
    X2 = *RK++ ^ RT0[ ( Y2       ) & 0xFF ] ^   \
                 RT1[ ( Y1 >>  8 ) & 0xFF ] ^   \
                 RT2[ ( Y0 >> 16 ) & 0xFF ] ^   \
                 RT3[ ( Y3 >> 24 ) & 0xFF ];    \
                                                \
    X3 = *RK++ ^ RT0[ ( Y3       ) & 0xFF ] ^   \
                 RT1[ ( Y2 >>  8 ) & 0xFF ] ^   \
                 RT2[ ( Y1 >> 16 ) & 0xFF ] ^   \
                 RT3[ ( Y0 >> 24 ) & 0xFF ];    \
}

int a_crypt_ecb( a_ctx *ctx,
                    int mode,
                    const uchar input[16],
                    uchar output[16] )
{
    int i;
    ulong *RK, X0, X1, X2, X3, Y0, Y1, Y2, Y3;

    RK = ctx->rk;

    G_U_LE( X0, input,  0 ); X0 ^= *RK++;
    G_U_LE( X1, input,  4 ); X1 ^= *RK++;
    G_U_LE( X2, input,  8 ); X2 ^= *RK++;
    G_U_LE( X3, input, 12 ); X3 ^= *RK++;

    /* ----------- */
    /* AES_DECRYPT */
    /* ----------- */   
    if( mode == AES_DECRYPT )
    {
        for( i = (ctx->nr >> 1) - 1; i > 0; i-- )
        {
            A_R( Y0, Y1, Y2, Y3, X0, X1, X2, X3 );
            A_R( X0, X1, X2, X3, Y0, Y1, Y2, Y3 );
        }

        A_R( Y0, Y1, Y2, Y3, X0, X1, X2, X3 );

        X0 = *RK++ ^ \
                ( (ulong) RSb[ ( Y0       ) & 0xFF ]       ) ^
                ( (ulong) RSb[ ( Y3 >>  8 ) & 0xFF ] <<  8 ) ^
                ( (ulong) RSb[ ( Y2 >> 16 ) & 0xFF ] << 16 ) ^
                ( (ulong) RSb[ ( Y1 >> 24 ) & 0xFF ] << 24 );

        X1 = *RK++ ^ \
                ( (ulong) RSb[ ( Y1       ) & 0xFF ]       ) ^
                ( (ulong) RSb[ ( Y0 >>  8 ) & 0xFF ] <<  8 ) ^
                ( (ulong) RSb[ ( Y3 >> 16 ) & 0xFF ] << 16 ) ^
                ( (ulong) RSb[ ( Y2 >> 24 ) & 0xFF ] << 24 );

        X2 = *RK++ ^ \
                ( (ulong) RSb[ ( Y2       ) & 0xFF ]       ) ^
                ( (ulong) RSb[ ( Y1 >>  8 ) & 0xFF ] <<  8 ) ^
                ( (ulong) RSb[ ( Y0 >> 16 ) & 0xFF ] << 16 ) ^
                ( (ulong) RSb[ ( Y3 >> 24 ) & 0xFF ] << 24 );

        X3 = *RK++ ^ \
                ( (ulong) RSb[ ( Y3       ) & 0xFF ]       ) ^
                ( (ulong) RSb[ ( Y2 >>  8 ) & 0xFF ] <<  8 ) ^
                ( (ulong) RSb[ ( Y1 >> 16 ) & 0xFF ] << 16 ) ^
                ( (ulong) RSb[ ( Y0 >> 24 ) & 0xFF ] << 24 );
    }
    else /* AES_ENCRYPT */
    {
        for( i = (ctx->nr >> 1) - 1; i > 0; i-- )
        {
            A_F( Y0, Y1, Y2, Y3, X0, X1, X2, X3 );
            A_F( X0, X1, X2, X3, Y0, Y1, Y2, Y3 );
        }

        A_F( Y0, Y1, Y2, Y3, X0, X1, X2, X3 );

        X0 = *RK++ ^ \
                ( (ulong) FSb[ ( Y0       ) & 0xFF ]       ) ^
                ( (ulong) FSb[ ( Y1 >>  8 ) & 0xFF ] <<  8 ) ^
                ( (ulong) FSb[ ( Y2 >> 16 ) & 0xFF ] << 16 ) ^
                ( (ulong) FSb[ ( Y3 >> 24 ) & 0xFF ] << 24 );

        X1 = *RK++ ^ \
                ( (ulong) FSb[ ( Y1       ) & 0xFF ]       ) ^
                ( (ulong) FSb[ ( Y2 >>  8 ) & 0xFF ] <<  8 ) ^
                ( (ulong) FSb[ ( Y3 >> 16 ) & 0xFF ] << 16 ) ^
                ( (ulong) FSb[ ( Y0 >> 24 ) & 0xFF ] << 24 );

        X2 = *RK++ ^ \
                ( (ulong) FSb[ ( Y2       ) & 0xFF ]       ) ^
                ( (ulong) FSb[ ( Y3 >>  8 ) & 0xFF ] <<  8 ) ^
                ( (ulong) FSb[ ( Y0 >> 16 ) & 0xFF ] << 16 ) ^
                ( (ulong) FSb[ ( Y1 >> 24 ) & 0xFF ] << 24 );

        X3 = *RK++ ^ \
                ( (ulong) FSb[ ( Y3       ) & 0xFF ]       ) ^
                ( (ulong) FSb[ ( Y0 >>  8 ) & 0xFF ] <<  8 ) ^
                ( (ulong) FSb[ ( Y1 >> 16 ) & 0xFF ] << 16 ) ^
                ( (ulong) FSb[ ( Y2 >> 24 ) & 0xFF ] << 24 );
    }

    P_U_LE( X0, output,  0 );
    P_U_LE( X1, output,  4 );
    P_U_LE( X2, output,  8 );
    P_U_LE( X3, output, 12 );

    return( 0 );
}

int a_crypt_cbc( a_ctx *ctx,
                    int mode,
                    size_t length,
                    uchar iv[16],
                    const uchar *input,
                    uchar *output )
{
    int i;
    uchar temp[16];

    if( length % 16 )
        return( E_AES_INVALID_INPUT_LENGTH );

    if( mode == AES_DECRYPT )
    {
        while( length > 0 )
        {
            memcpy( temp, input, 16 );
            a_crypt_ecb( ctx, mode, input, output );

            for( i = 0; i < 16; i++ )
                output[i] = (uchar)( output[i] ^ iv[i] );

            memcpy( iv, temp, 16 );

            input  += 16;
            output += 16;
            length -= 16;
        }
    }
    else
    {
        while( length > 0 )
        {
            for( i = 0; i < 16; i++ )
                output[i] = (uchar)( input[i] ^ iv[i] );

            a_crypt_ecb( ctx, mode, output, output );
            memcpy( iv, output, 16 );

            input  += 16;
            output += 16;
            length -= 16;

        }
    }

    return( 0 );
}

/**************************************************************************
 *  SO FUNCTION - ENCRYPTION
 **************************************************************************/
int aes_so_enc (uchar* ip_buf,  uint32 ip_len, uchar* op_buf, uint32 op_len)
{
    uint32 i = 0;
    uint32 ret = 0;

    if (ip_len != op_len)
    {
        SMSG(true,"[%s] error, ip len should be equal to op len\n",MOD);
        return -1;
    }

    if (0 != ip_len % CIPHER_BLOCK_SIZE)
    {
        SMSG(true,"[%s] error, ip len should be mutiple of %d bytes\n",MOD,CIPHER_BLOCK_SIZE);
        return -1;
    }


    if(0 == g_AES_Key[0])
    {
        SMSG(true,"[%s] Enc Key Is ZERO. Fail\n",MOD);
        goto _err;    
    }

    ret = a_enc(&aes, (uchar*)g_AES_Key, aes_key_len*8);

    if (ret != 0) 
    {
        SMSG(true,"a_enc error -%02X\n", -ret);
        goto _err;
    }    

    for (i = 0; i!=ip_len ; i+=CIPHER_BLOCK_SIZE)
    {
        ret = a_crypt_cbc(&aes, AES_ENCRYPT, CIPHER_BLOCK_SIZE, (uchar*)g_AES_IV_TEMP, ip_buf + i, op_buf + i);
        if (ret != 0)
        {
            SMSG(true,"hairtunes: a_cbc error -%02X\n", -ret);
            goto _err;
        }
    }

    return 0;

_err:

    return -1;
}

/**************************************************************************
 *  SO FUNCTION - DECRYPTION
 **************************************************************************/
int aes_so_dec (uchar* ip_buf,  uint32 ip_len, uchar* op_buf, uint32 op_len)
{
    uint32 i = 0;
    uint32 ret = 0;    

    if (ip_len != op_len)
    {
        SMSG(true,"[%s] error, ip len should be equal to op len\n",MOD);
        return -1;
    }

    if (0 != ip_len % CIPHER_BLOCK_SIZE)
    {
        SMSG(true,"[%s] error, ip len should be mutiple of %d bytes\n",MOD,CIPHER_BLOCK_SIZE);
        return -1;
    }

    if(0 == g_AES_Key[0])
    {
        SMSG(true,"[%s] Dec Key Is ZERO. Fail\n",MOD);
        goto _err;    
    }

    ret = a_dec(&aes, (uchar*)g_AES_Key, aes_key_len*8);
    if (ret != 0) 
    {
        SMSG(true,"a_dec error -%02X\n", -ret);
        goto _err;
    }    

    for (i = 0; i!=ip_len ; i+=CIPHER_BLOCK_SIZE)
    {
        ret = a_crypt_cbc(&aes, AES_DECRYPT, 0x10, (uchar*)g_AES_IV_TEMP, ip_buf + i, op_buf + i);
        if (ret != 0)
        {
            SMSG(true,"hairtunes: a_cbc error -%02X\n", -ret);
            goto _err;
        }
    }
    
    return 0;

_err:

    return -1;

}

/**************************************************************************
 *  SO FUNCTION - KEY INITIALIZATION
 **************************************************************************/
/* WARNING ! this function is not the same as cipher tool */
int aes_so_init_key (uchar* key_buf,  uint32 key_len)
{
    uint32 i = 0;
    uchar temp[CT_AES128_LEN*2];
    uint32 n = 0;
    uint32 val = 0;
    uchar c;
    int j = 0;    
	uchar fmt_str[2] = {0};	


    if(0 == key_buf)
    {
        SMSG(true,"[%s] Init Key Is ZERO. Fail\n",MOD);
        goto _err;    
    }

    /* -------------------------------------------------- */
    /* check key length                                   */
    /* -------------------------------------------------- */
    switch(key_len)
    {
        case CT_AES128_LEN:
            break;        
        case CT_AES192_LEN:
        case CT_AES256_LEN:
            SMSG(true,"[%s] Only AES 128 is supported\n",MOD);
            goto _err;        
        default:
            SMSG(true,"[%s] Len Invalid %d\n",MOD,key_len);
            goto _err;
    }

    aes_key_len = key_len;

    /* -------------------------------------------------- */
    /* copy key to temporarily buffer                     */    
    /* -------------------------------------------------- */
    mcpy(temp,key_buf,CT_AES128_LEN*2);

    /* -------------------------------------------------- */
    /* revert string to accomodate OpenSSL format         */
    /* -------------------------------------------------- */
    for(i=0;i<key_len*2;i+=8)
    {
        c               = temp[i];
        temp[i]         = temp[i+6];
        temp[i+6]       = c;
        c               = temp[i+1];
        temp[i+1]       = temp[i+7];
        temp[i+7]       = c;

        c               = temp[i+2];
        temp[i+2]       = temp[i+4];
        temp[i+4]       = c;
        c               = temp[i+3];
        temp[i+3]       = temp[i+5];
        temp[i+5]       = c;
    }

    /* -------------------------------------------------- */
    /* convert key value from string format to hex format */
    /* -------------------------------------------------- */
  
    i = 0;
    n = 0;
    
    while(n < key_len*2)
    {

        for(j=0; j<8; j++)
        {
            fmt_str[0] = temp[n+j];
            sscanf(fmt_str,"%x",&val);
            g_AES_Key[i] = g_AES_Key[i]*16;
            g_AES_Key[i] += val;             
        }

        /* get next key value */
        i ++;
        n += 8;
    }    

    /* -------------------------------------------------- */
    /* reinit IV                                          */
    /* -------------------------------------------------- */
    for(i=0;i<4;i++)
    {
        g_AES_IV_TEMP[i] = g_AES_IV[i];
    }

    /* dump information for debugging */
    for(i=0; i<1; i++)
    {
        SMSG(true,"0x%x\n",g_AES_Key[i]);
    }

    for(i=0; i<1; i++)
    {
        SMSG(true,"0x%x\n",g_AES_IV_TEMP[i]);
    }

    return 0;

_err:

    return -1;

}

/**************************************************************************
 *  SO FUNCTION - VECTOR INITIALIZATION
 **************************************************************************/
int aes_so_init_vector (void)
{
    uint32 i = 0;
    
    /* -------------------------------------------------- */
    /* reinit IV                                          */
    /* -------------------------------------------------- */
    for(i=0;i<4;i++)
    {
        g_AES_IV_TEMP[i] = g_AES_IV[i];
    }

    /* dump information for debugging */
    for(i=0; i<1; i++)
    {
        SMSG(true,"0x%x\n",g_AES_Key[i]);
    }

    for(i=0; i<1; i++)
    {
        SMSG(true,"0x%x\n",g_AES_IV_TEMP[i]);
    }

    return 0;
}


