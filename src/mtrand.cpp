// Copyright 2021 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*****************************************************************************************************************
 *
 * 	 Authors: Hassan Umari, Pablo Inigo Blasco
 *
 ******************************************************************************************************************/

// mtrand.cpp, see include file mtrand.h for information

#include "mtrand.h"
// non-inline function definitions and static member definitions cannot
// reside in header file because of the risk of multiple declarations

// initialization of static private members
unsigned long MTRand_int32::state[n] = {0x0UL};
int MTRand_int32::p = 0;
bool MTRand_int32::init = false;

void MTRand_int32::gen_state() { // generate new state vector
  for (int i = 0; i < (n - m); ++i)
    state[i] = state[i + m] ^ twiddle(state[i], state[i + 1]);
  for (int i = n - m; i < (n - 1); ++i)
    state[i] = state[i + m - n] ^ twiddle(state[i], state[i + 1]);
  state[n - 1] = state[m - 1] ^ twiddle(state[n - 1], state[0]);
  p = 0; // reset position
}

void MTRand_int32::seed(unsigned long s) { // init by 32 bit seed
  state[0] = s & 0xFFFFFFFFUL;             // for > 32 bit machines
  for (int i = 1; i < n; ++i) {
    state[i] = 1812433253UL * (state[i - 1] ^ (state[i - 1] >> 30)) + i;
    // see Knuth TAOCP Vol2. 3rd Ed. P.106 for multiplier
    // in the previous versions, MSBs of the seed affect only MSBs of the array
    // state 2002/01/09 modified by Makoto Matsumoto
    state[i] &= 0xFFFFFFFFUL; // for > 32 bit machines
  }
  p = n; // force gen_state() to be called for next random number
}

void MTRand_int32::seed(const unsigned long *array, int size) { // init by array
  seed(19650218UL);
  int i = 1, j = 0;
  for (int k = ((n > size) ? n : size); k; --k) {
    state[i] =
        (state[i] ^ ((state[i - 1] ^ (state[i - 1] >> 30)) * 1664525UL)) +
        array[j] + j;         // non linear
    state[i] &= 0xFFFFFFFFUL; // for > 32 bit machines
    ++j;
    j %= size;
    if ((++i) == n) {
      state[0] = state[n - 1];
      i = 1;
    }
  }
  for (int k = n - 1; k; --k) {
    state[i] =
        (state[i] ^ ((state[i - 1] ^ (state[i - 1] >> 30)) * 1566083941UL)) - i;
    state[i] &= 0xFFFFFFFFUL; // for > 32 bit machines
    if ((++i) == n) {
      state[0] = state[n - 1];
      i = 1;
    }
  }
  state[0] = 0x80000000UL; // MSB is 1; assuring non-zero initial array
  p = n; // force gen_state() to be called for next random number
}
