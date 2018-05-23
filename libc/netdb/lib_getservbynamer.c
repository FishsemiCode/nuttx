/****************************************************************************
 * libc/netdb/lib_getservbynamer.c
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

#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>

#include "libc.h"
#include "netdb/lib_netdb.h"

#ifdef CONFIG_LIBC_NETDB

#define MAXSERVS 2
#define ALIGN (sizeof(struct { char a; char *b; }) - sizeof(char *))

struct service
{
  uint16_t port;
  unsigned char proto, socktype;
};

static int __lookup_serv(struct service buf[MAXSERVS],
        const char *name, int proto, int socktype, int flags)
{
  char line[128];
  int cnt = 0;
  char *p, *z = "";
  unsigned long port = 0;

  switch (socktype)
    {
    case SOCK_STREAM:
      switch (proto)
        {
        case 0:
            proto = IPPROTO_TCP;
        case IPPROTO_TCP:
            break;
        default:
            return EAI_SERVICE;
        }
      break;
    case SOCK_DGRAM:
      switch (proto)
        {
        case 0:
            proto = IPPROTO_UDP;
        case IPPROTO_UDP:
            break;
        default:
            return EAI_SERVICE;
        }
    default:
      if (name) return EAI_SERVICE;
      buf[0].port = 0;

      buf[0].proto = proto;
      buf[0].socktype = socktype;
      return 1;
    }

  if (name)
    {
      if (!*name) return EAI_SERVICE;
      port = strtoul(name, &z, 10);
    }
  if (!*z)
    {
      if (port > 65535) return EAI_SERVICE;
      if (proto != IPPROTO_UDP)
        {
          buf[cnt].port = port;
          buf[cnt].socktype = SOCK_STREAM;
          buf[cnt++].proto = IPPROTO_TCP;
        }
      if (proto != IPPROTO_TCP)
        {
          buf[cnt].port = port;
          buf[cnt].socktype = SOCK_DGRAM;
          buf[cnt++].proto = IPPROTO_UDP;
        }
      return cnt;
    }

  if (flags & AI_NUMERICSERV) return EAI_NONAME;

  size_t l = strlen(name);

  FILE *f = fopen(CONFIG_NETDB_SERVICES_PATH, "rb");
  if (!f) switch (errno)
    {
      case ENOENT:
      case ENOTDIR:
      case EACCES:
          return EAI_SERVICE;
      default:
          return EAI_SYSTEM;
    }

  while (fgets(line, sizeof line, f) && cnt < MAXSERVS)
    {
      if ((p=strchr(line, '#'))) *p++='\n', *p=0;

      /* Find service name */
      for(p=line; (p=strstr(p, name)); p++)
        {
          if (p>line && !isspace(p[-1])) continue;
          if (p[l] && !isspace(p[l])) continue;
          break;
        }
      if (!p) continue;

      /* Skip past canonical name at beginning of line */
      for (p=line; *p && !isspace(*p); p++);

      port = strtoul(p, &z, 10);
      if (port > 65535 || z==p) continue;
      if (!strncmp(z, "/udp", 4))
        {
          if (proto == IPPROTO_TCP) continue;
          buf[cnt].port = port;
          buf[cnt].socktype = SOCK_DGRAM;
          buf[cnt++].proto = IPPROTO_UDP;
        }
      if (!strncmp(z, "/tcp", 4))
        {
          if (proto == IPPROTO_UDP) continue;
          buf[cnt].port = port;
          buf[cnt].socktype = SOCK_STREAM;
          buf[cnt++].proto = IPPROTO_TCP;
        }
    }
  fclose(f);
  return cnt > 0 ? cnt : EAI_SERVICE;
}

/****************************************************************************
 * Name: getservname_r
 *
 * Description:
 *   The getservbyname_r() function returns a structure of type servent for
 *   the given serv name. Here name is either a servname, or an IPv4 address
 *   in standard dot notation (as for inet_addr(3)), or an IPv6 address in
 *   colon (and possibly dot) notation.
 *
 *   If name is an IPv4 or IPv6 address, no lookup is performed and
 *   getservbyname_r() simply copies name into the h_name field
 *   and its struct in_addr equivalent into the h_addr_list[0] field of the
 *   returned servent structure.
 *
 *   getservname_r() is *not* POSIX but is similar to a Glibc extension and is
 *   used internally by NuttX to implement the POSIX getservname().
 *
 * Input Parameters:
 *   name - The name of the serv to find.
 *   prots - The protocol to use.
 *   serv - Caller provided location to return the serv data.
 *   buf - Caller provided buffer to hold string data associated with the
 *     serv data.
 *   buflen - The size of the caller-provided buffer
 *   h_errnop - There h_errno value returned in the event of a failure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success, -1 (ERROR) is returned on a failure
 *   with the returned h_errno value provided the reason for the failure.
 *
 ****************************************************************************/

int getservbyname_r(FAR const char *name, FAR const char *proto_,
                    FAR struct servent *serv, FAR char *buf, size_t buflen,
                    int *h_errnop)
{
  struct service servs[MAXSERVS];
  int cnt, proto, align;

  /* Don't treat numeric port number strings as service records. */
  char *end = "";
  strtoul(name, &end, 10);
  if (!*end) return ENOENT;

  /* Align buffer */
  align = -(uintptr_t)buf & (ALIGN-1);
  if (buflen < 2*sizeof(char *)+align)
      return ERANGE;
  buf += align;

  if (!proto_) proto = 0;
  else if (!strcmp(proto_, "tcp")) proto = IPPROTO_TCP;
  else if (!strcmp(proto_, "udp")) proto = IPPROTO_UDP;
  else return EINVAL;

  cnt = __lookup_serv(servs, name, proto, 0, 0);
  if (cnt<0) switch (cnt)
  {
      case EAI_MEMORY:
      case EAI_SYSTEM:
          return ENOMEM;
      default:
          return ENOENT;
  }

  serv->s_name = (char *)name;
  serv->s_aliases = (void *)buf;
  serv->s_aliases[0] = serv->s_name;
  serv->s_aliases[1] = 0;
  serv->s_port = htons(servs[0].port);
  serv->s_proto = servs[0].proto == IPPROTO_TCP ? "tcp" : "udp";

  return 0;
}

#endif /* CONFIG_LIBC_NETDB */
