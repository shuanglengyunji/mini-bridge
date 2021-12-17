/**
 * @file
 * HTTPD simple SSI example
 *
 * This file demonstrates how to add support for SSI.
 * It does this in a very simple way by providing the three tags 'HelloWorld'
 * 'counter', and 'MultiPart'.
 *
 * This file also demonstrates how to integrate CGI with SSI.
 */
 
 /*
 * Copyright (c) 2017 Simon Goldschmidt
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Simon Goldschmidt <goldsimon@gmx.de>
 *
 */

#include "lwip/opt.h"
// #include "ssi_example.h"

#include "lwip/apps/httpd.h"

#include "lwip/def.h"
#include "lwip/mem.h"

#include <stdio.h>
#include <string.h>

/** define LWIP_HTTPD_EXAMPLE_SSI_SIMPLE to 1 to enable this ssi example*/
#ifndef LWIP_HTTPD_EXAMPLE_SSI_SIMPLE
#define LWIP_HTTPD_EXAMPLE_SSI_SIMPLE 0
#endif

#define LWIP_HTTPD_EXAMPLE_SSI_SIMPLE 1
#if LWIP_HTTPD_EXAMPLE_SSI_SIMPLE

const char * ssi_example_tags[] = {
  "HellWorl",
  "counter",
  "MultPart"
};

#define LWIP_HTTPD_SSI_RAW 1
u16_t ssi_example_ssi_handler(
                             const char* ssi_tag_name,
                             char *pcInsert, int iInsertLen
                             )
{
  size_t printed;
#if LWIP_HTTPD_SSI_RAW
  /* a real application could use if(!strcmp) blocks here, but we want to keep
     the differences between configurations small, so translate string to index here */
  int iIndex;
  for (iIndex = 0; iIndex < LWIP_ARRAYSIZE(ssi_example_tags); iIndex++) {
    if(!strcmp(ssi_tag_name, ssi_example_tags[iIndex])) {
      break;
    }
  }
#endif
  switch (iIndex) {
  case 0: /* "HelloWorld" */
    printed = snprintf(pcInsert, iInsertLen, "Hello World!");
    break;
  case 1: /* "counter" */
    {
      static int counter;
      counter++;
      printed = snprintf(pcInsert, iInsertLen, "%d", counter);
    }
    break;
  case 2: /* "MultPart" */
    printed = snprintf(pcInsert, iInsertLen, "LWIP_HTTPD_SSI_MULTIPART disabled");
    break;
  default: /* unknown tag */
    printed = 0;
    break;
  }
  LWIP_ASSERT("sane length", printed <= 0xFFFF);
  return (u16_t)printed;
}

void
ssi_ex_init(void)
{
  int i;

  http_set_ssi_handler(ssi_example_ssi_handler,
    NULL, 0
    );
}

#endif /* LWIP_HTTPD_EXAMPLE_SSI_SIMPLE */
