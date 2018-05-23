/****************************************************************************
 * libc/netdb/lib_getservbyname.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Guiding Li <ligduiding@pinecone.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <netdb.h>
#include <errno.h>

#include "libc.h"
#include "netdb/lib_netdb.h"

#ifdef CONFIG_LIBC_NETDB

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getservname
 *
 * Description:
 *   The getservbyname() function returns a structure of type servent for
 *   the given serv name. Here name is either a servname, or an IPv4 address
 *   in standard dot notation (as for inet_addr(3)), or an IPv6 address in
 *   colon (and possibly dot) notation.
 *
 *   If name is an IPv4 or IPv6 address, no lookup is performed and
 *   getservbyname_r() simply copies name into the h_name field
 *   and its struct in_addr equivalent into the h_addr_list[0] field of the
 *   returned servent structure.
 *
 * Input Parameters:
 *   name  - The name of the serv to find.
 *   proto - The protocol to use.
 *
 * Returned Value:
 *   Upon successful completion, this function will return a pointer to a
 *   servent structure if the requested entry was found, and a null pointer
 *   if the end of the database was reached or the requested entry was not
 *   found.
 *
 *   Upon unsuccessful completion, getservbyname() will set h_errno to
 *   indicate the error
 *
 ****************************************************************************/

FAR struct servent *getservbyname(FAR const char *name, FAR const char *proto)
{
  int ret;

  DEBUGASSERT(name != NULL);
  ret = getservbyname_r(name, proto, &g_servent, g_servbuffer,
                        CONFIG_NETDB_BUFSIZE, &h_errno);
  return ret == 0 ? &g_servent : NULL;
}

#endif /* CONFIG_LIBC_NETDB */
