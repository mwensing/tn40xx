# tn40xx
FreeBSD tn40xx Driver - See Copyright Below

# Notes

2023-03-06 - Compiled on FreeBSD 12.3 - Still need to test versus the .ko I found
             at [1] https://www.uptimed.nl/drivers for FreeBSD. The .ko at that site
             does run, so I need to compare that with the one I have built from these
             sources. The .ko at [1] is running in my Proxmox virtualized Pfsense 2.6.0 
             install (in this case) so I have a working env to test with and report back on

2023-03-04 - First code copy and paste salvage effort 
           - See the History below
           - See also the Copyright attribution
           - At this time it is unclear if this will build, but I will try that next
           - I also found https://github.com/acooks/tn40xx-driver so some of what is
             there might also help here.

# History

I decided to see if I could salvage the tn40xx driver for FreeBSD

After much google foo I found:

https://reviews.freebsd.org/D19433#change-nPKHzane1gJX

At the time this was: 

~kevans/tn40xx-build.diff

Added tn40xx

Changes after first round of tests.

Support FreeBSD 12.x and other fixes

Version 1.1

Bug fix.

Ver 1.2: Added README file. Added license header.

---

# Copyright

The code within is mostly :
/*
 * Copyright (c) 2019 Tehuti Networks Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
