# HydraSDR Library Licenses

The HydraSDR library includes components with different licenses.
Both licenses are permissive and compatible with each other.

## MIT License

Most of the library is licensed under the MIT License:

**Files by Benjamin Vernoux:**
- src/iqconverter.c, src/iqconverter.h
- src/iqconverter_lut.c, src/iqconverter_lut.h
- src/iqconverter_decimator.c, src/iqconverter_decimator.h
- src/iqconverter_float_opt_dec.c, src/iqconverter_float_opt_dec.h
- src/iqconverter_int16_opt_dec.c, src/iqconverter_int16_opt_dec.h
- src/iqconverter_common.h
- src/compat_opt.h
- src/filters_opt.h
- src/spsc_queue.h
- src/triple_buffer.h
- src/buffer_pool.h
- src/hydrasdr_shared.c, src/hydrasdr_shared.h
- src/hydrasdr_internal.h
- src/hydrasdr_commands.h
- src/hydrasdr_rfone.c

**Files by Youssef Touil:**
- src/filters.h
- src/iqconverter_float.c, src/iqconverter_float.h
- src/iqconverter_int16.c, src/iqconverter_int16.h

**Files by Youssef Touil and Benjamin Vernoux:**
- src/iqconverter_float_opt.c, src/iqconverter_float_opt.h
- src/iqconverter_float_opt33.c, src/iqconverter_float_opt33.h
- src/iqconverter_float_opt65.c, src/iqconverter_float_opt65.h
- src/iqconverter_float_opt83.c, src/iqconverter_float_opt83.h
- src/iqconverter_int16_opt.c, src/iqconverter_int16_opt.h
- src/iqconverter_int16_opt33.c, src/iqconverter_int16_opt33.h
- src/iqconverter_int16_opt65.c, src/iqconverter_int16_opt65.h
- src/iqconverter_int16_opt83.c, src/iqconverter_int16_opt83.h

```
MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
```

## BSD-3-Clause License

The following files are licensed under the BSD-3-Clause License due to
contributions from multiple authors:

**Files by Jared Boone, Michael Ossmann, Youssef Touil, Ian Gilmour, and Benjamin Vernoux:**
- src/hydrasdr.c
- src/hydrasdr.h

```
BSD-3-Clause License

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of HydraSDR nor the names of its contributors may be used
   to endorse or promote products derived from this software without specific
   prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```
