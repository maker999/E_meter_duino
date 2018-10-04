#include "Arduino.h"
#include "crypto_diffie_hellman.h"
/*
#include "lib/basic/atca_basic.h"
#include "lib/atca_cfgs.h"
#include "lib/atca_command.h"
#include "lib/atca_compiler.h"
#include "lib/crypto/atca_crypto_sw.h"
#include "lib/crypto/atca_crypto_sw_sha2.h"
#include "lib/atca_device.h"
#include "lib/atca_devtypes.h"
#include "lib/hal/atca_hal.h"
#include "lib/basic/atca_helpers.h"
#include "lib/host/atca_host.h"
#include "lib/atca_iface.h"
#include "lib/hal/atca_start_config.h"
#include "lib/hal/atca_start_iface.h"
#include "lib/atca_status.h"
#include "lib/cryptoauthlib.h"
#include "lib/hal/hal_i2c_bitbang.h"
#include <i2c_bitbang_at88ck9000.h>
#include "lib/crypto/hashes/sha2_routines.h"
#include <uECC.h>
*/


/*#define uECC_OPTIMIZATION_LEVEL 3
#define uECC_PLATFORM   uECC_arm
#define uECC_arm        3
#define uECC_WORD_SIZE  4
#define uECC_SUPPORTS_secp256r1
#define uECC_asm uECC_asm_fast*/
#define BAUD_RATE 9600



uint8_t uECC_RNG_ZEROIZE[32] = {0x00};
int pubKeyHostCnt = 0;

ATCAIfaceCfg *gCfg = &cfg_ateccx08a_i2c_default;
/*
ATCAIfaceCfg gCfg = {.iface_type = ATCA_I2C_IFACE,
                      .devtype = ATECC508A,
                      .atcai2c.slave_address = 0xC0,
                      .atcai2c.bus = 0,
                      .atcai2c.baud = 400000,
                      .wake_delay = 1500,
                      .rx_retries = 20
};*/
uint8_t buffer[64];
ATCA_STATUS status = ATCA_GEN_FAIL;

uint8_t uECC_RNG_PARAMETERS[32] = {
           0x82,0xc9,0x00,0x87,0xeb,0x71,0x1a,0x35,0x15,0x80,0xcc,0x72,0x61,0x73,0x8b,0xcb,
           0xb6,0x5a,0xbe,0x33,0xe1,0xe3,0x70,0x19,0x0e,0xe7,0x4f,0xd7,0x94,0x21,0xc8,0xc4
};

uint8_t ecc_pubKeyHost[64] =  {
           0x12,0x77,0x00,0x87,0xeb,0x71,0x1a,0x35,0x15,0x80,0xcc,0x72,0x61,0x73,0x8b,0xcb,
           0xb6,0x5a,0xbe,0x33,0xe1,0xe3,0x70,0x19,0x0e,0xe7,0x4f,0xd7,0x94,0x21,0xad,0x82,
           0x44,0x77,0x00,0x87,0xeb,0x71,0x1a,0x35,0x15,0x80,0xcc,0x72,0x61,0x73,0x8b,0xcb,
           0xb6,0x5a,0xbe,0x33,0xe1,0xe3,0x70,0x19,0x0e,0xe7,0x4f,0xd7,0x94,0x21,0xad,0x21
};


 
static int TRNG_fromSeed(uint8_t *dest, unsigned size) {
  memcpy(dest, uECC_RNG_PARAMETERS, 32);
  return 1;
}

/*
static int TRNG(uint8_t *dest, unsigned size) {
  uint8_t rand_out[RANDOM_RSP_SIZE];
  status = atcab_init( gCfg );
  status = atcab_random(rand_out);
  status = atcab_release();
  //delayMicroseconds(20);
  delay(80);
  memcpy(dest, rand_out, 32);
  return 1;
}


uint8_t* secretASM() {
	uint8_t ecc_privKeyASM[32];
	uint8_t ecdh_secretASM[32];
  uint8_t ecc_pubKeyASM[64]; 
	uECC_set_rng(&TRNG_fromSeed);	
	const struct uECC_Curve_t * curve = uECC_secp256r1();	
	uECC_make_key(ecc_pubKeyASM, ecc_privKeyASM, curve);
	uECC_shared_secret(ecc_pubKeyHost, ecc_privKeyASM, ecdh_secretASM, curve);
	return &ecdh_secretASM[0];
}*/

boolean testit(){
  status = atcab_init(gCfg);
 if(status != ATCA_SUCCESS) {
   return false;
 }  
 return true;
 //status = atcab_random(buffer);
 //if(status != ATCA_SUCCESS) {
 //  return false;
 //}
}

