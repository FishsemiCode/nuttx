/****************************************************************************
 * net/route/net_del_fileroute.c
 *
 *   Copyright (C) 2017-2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <arpa/inet.h>

#include <nuttx/fs/fs.h>
#include <nuttx/net/ip.h>

#include "route/fileroute.h"
#include "route/cacheroute.h"
#include "route/route.h"

#if defined(CONFIG_ROUTE_IPv4_FILEROUTE) || defined(CONFIG_ROUTE_IPv6_FILEROUTE)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
struct route_match_ipv4_s
{
  in_addr_t                    target;   /* The target IP address to match */
  in_addr_t                    netmask;  /* The network mask to match */
  unsigned int                 index;    /* Index of match */
};
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
struct route_match_ipv6_s
{
  net_ipv6addr_t               target;   /* The target IP address to match */
  net_ipv6addr_t               netmask;  /* The network mask to match */
  unsigned int                 index;    /* Index of match */
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_match_ipv4
 *
 * Description:
 *   Return 1 if the route is available
 *
 * Input Parameters:
 *   route - The next route to examine
 *   arg   - The match values (cast to void*)
 *
 * Returned Value:
 *   0 if the entry is not a match; 1 if the entry matched and was cleared.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
static int net_match_ipv4(FAR struct net_route_ipv4_s *route, FAR void *arg)
{
  FAR struct route_match_ipv4_s *match = (FAR struct route_match_ipv4_s *)arg;

  /* To match, the masked target address must be the same, and the masks
   * must be the same.
   */

  net_ipv4_dumproute("Comparing", route);
  ninfo("With:\n");
  ninfo("  target=%08lx netmask=%08lx\n",
        htonl(match->target), htonl(match->netmask));

  if (net_ipv4addr_maskcmp(route->target, match->target, match->netmask) &&
      net_ipv4addr_cmp(route->netmask, match->netmask))
    {
      /* They match.. a non-zero value to terminate the traversal.  The last
       * value of index is the index to the matching entry.
       */

      return 1;
    }

  /* Next time we are here, this will be the routing table index */

  match->index++;
  return 0;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
static int net_match_ipv6(FAR struct net_route_ipv6_s *route, FAR void *arg)
{
  FAR struct route_match_ipv6_s *match = (FAR struct route_match_ipv6_s *)arg;

  /* To match, the masked target address must be the same, and the masks
   * must be the same.
   */

  net_ipv6_dumproute("Comparing", route);
  ninfo("With:\n");
  ninfo("  target:  %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        htons(match->target[0]),  htons(match->target[1]),
        htons(match->target[2]),  htons(match->target[3]),
        htons(match->target[4]),  htons(match->target[5]),
        htons(match->target[6]),  htons(match->target[7]));
  ninfo("  netmask: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        htons(match->netmask[0]), htons(match->netmask[1]),
        htons(match->netmask[2]), htons(match->netmask[3]),
        htons(match->netmask[4]), htons(match->netmask[5]),
        htons(match->netmask[6]), htons(match->netmask[7]));

  if (net_ipv6addr_maskcmp(route->target, match->target, match->netmask) &&
      net_ipv6addr_cmp(route->netmask, match->netmask))
    {
      /* They match.. a non-zero value to terminate the traversal.  The last
       * value of index is the index to the matching entry.
       */

      return 1;
    }

  /* Next time we are here, this will be the routing table index */

  match->index++;
  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_delroute_ipv4 and net_delroute_ipv6
 *
 * Description:
 *   Remove an existing route from the routing table
 *
 * Input Parameters:
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
int net_delroute_ipv4(in_addr_t target, in_addr_t netmask)
{
  struct route_match_ipv4_s match;
  struct net_route_ipv4_s route;
  struct file fshandle;
  off_t filesize;
  ssize_t nwritten;
  ssize_t nread;
  off_t pos;
  int nentries;
  int index;
  int ret;

  /* We must lock out other accesses to the routing table while we remove
   * entry
   */

  ret = net_lockroute_ipv4();
  if (ret < 0)
    {
       nerr("ERROR: net_lockroute_ipv4 failed: %d\n", ret);
       return ret;
    }

  /* Get the size of the routing table (in entries) */

  nentries = net_routesize_ipv4();
  if (nentries < 0)
    {
      ret = nentries;
      goto errout_with_lock;
    }
  else if (nentries == 0)
    {
      ret = -ENOENT;
      goto errout_with_lock;
    }

  /* Set up the comparison structure */

  net_ipv4addr_copy(match.target, target);
  net_ipv4addr_copy(match.netmask, netmask);
  match.index = 0;

  /* Then find the index into the routing table where the match can be found */

  ret = net_foreachroute_ipv4(net_match_ipv4, &match);
  if (ret < 0)
    {
      /* And error occurred */

      goto errout_with_lock;
    }
  else if (ret == 0)
    {
      /* No match found */

      ret = -ENOENT;
      goto errout_with_lock;
    }

  /* Open the routing table for read/write access */

  ret = net_openroute_ipv4(O_RDWR, &fshandle);
  if (ret < 0)
    {
       nerr("ERROR: Could not open IPv4 routing table: %d\n", ret);
       goto errout_with_lock;
    }

#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
  /* We are committed to modifying the routing table.  Flush the in-memory
   * routing table cache.
   */

  net_flushcache_ipv4();
#endif

  /* Loop, copying each entry, to the previous entry thus removing the entry
   * to be deleted.
   */

  for (index = match.index + 1; index < nentries; index++)
    {
      /* Seek to the current entry to be moved */

      pos = net_seekroute_ipv4(&fshandle, index);
      if (pos < 0)
        {
          nerr("ERROR: net_readroute_ipv4 failed: %ld\n", (long)pos);
          ret =(int)pos;
          goto errout_with_fshandle;
        }

      /* Read the routing table entry at this position */

      nread = net_readroute_ipv4(&fshandle, &route);
      if (nread < 0)
        {
          nerr("ERROR: net_readroute_ipv4 failed: %ld\n", (long)nread);
          ret = (int)nread;
          goto errout_with_fshandle;
        }
      else if (nread == 0)
        {
          nerr("ERROR: Unexpected end of file\n");
          ret = -EINVAL;
          goto errout_with_fshandle;
        }

      /* Seek to the previous entry to be replaced */

      pos = net_seekroute_ipv4(&fshandle, index - 1);
      if (pos < 0)
        {
          nerr("ERROR: net_readroute_ipv4 failed: %ld\n", (long)pos);
          ret =(int)pos;
          goto errout_with_fshandle;
        }

      /* Now write the record to its new location */

      nwritten = net_writeroute_ipv4(&fshandle, &route);
      if (nwritten < 0)
        {
          nerr("ERROR: net_readroute_ipv4 failed: %ld\n", (long)nwritten);
          ret = (int)nwritten;
          goto errout_with_fshandle;
        }
    }

  /* Now truncate the one duplicate entry at the end of the file.  This may
   * result in a zero length file.
   */

  filesize = (nentries - 1) * sizeof(struct net_route_ipv4_s);
  ret = file_truncate(&fshandle, filesize);

errout_with_fshandle:
  (void)net_closeroute_ipv4(&fshandle);

errout_with_lock:
  (void)net_unlockroute_ipv4();
  return ret;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
int net_delroute_ipv6(net_ipv6addr_t target, net_ipv6addr_t netmask)
{
  struct route_match_ipv6_s match;
  struct net_route_ipv6_s route;
  struct file fshandle;
  off_t filesize;
  ssize_t nwritten;
  ssize_t nread;
  off_t pos;
  int nentries;
  int index;
  int ret;

  /* We must lock out other accesses to the routing table while we remove
   * entry
   */

  ret = net_lockroute_ipv6();
  if (ret < 0)
    {
       nerr("ERROR: net_lockroute_ipv6 failed: %d\n", ret);
       return ret;
    }

  /* Get the size of the routing table (in entries) */

  nentries = net_routesize_ipv6();
  if (nentries < 0)
    {
      ret = nentries;
      goto errout_with_lock;
    }
  else if (nentries == 0)
    {
      ret = -ENOENT;
      goto errout_with_lock;
    }

  /* Set up the comparison structure */

  net_ipv6addr_copy(match.target, target);
  net_ipv6addr_copy(match.netmask, netmask);
  match.index = 0;

  /* Then find the index into the routing table where the match can be found */

  ret = net_foreachroute_ipv6(net_match_ipv6, &match);
  if (ret < 0)
    {
      /* And error occurred */

      ret = ret;
      goto errout_with_lock;
    }
  else if (ret == 0)
    {
      /* No match found */

      ret = -ENOENT;
      goto errout_with_lock;
    }

  /* Open the routing table for read/write access */

  ret = net_openroute_ipv6(O_RDWR, &fshandle);
  if (ret < 0)
    {
      nerr("ERROR: Could not open IPv6 routing table: %d\n", ret);
      goto errout_with_lock;
    }

#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
  /* We are committed to modifying the routing table.  Flush the in-memory
   * routing table cache.
   */

  net_flushcache_ipv6();
#endif

  /* Loop, copying each entry, to the previous entry thus removing the entry
   * to be deleted.
   */

  for (index = match.index + 1; index < nentries; index++)
    {
      /* Seek to the current entry to be moved */

      pos = net_seekroute_ipv6(&fshandle, index);
      if (pos < 0)
        {
          nerr("ERROR: net_readroute_ipv6 failed: %ld\n", (long)pos);
          ret =(int)pos;
          goto errout_with_fshandle;
        }

      /* Read the routing table entry at this position */

      nread = net_readroute_ipv6(&fshandle, &route);
      if (nread < 0)
        {
          nerr("ERROR: net_readroute_ipv6 failed: %ld\n", (long)nread);
          ret = (int)nread;
          goto errout_with_fshandle;
        }
      else if (nread == 0)
        {
          nerr("ERROR: Unexpected end of file\n");
          ret = -EINVAL;
          goto errout_with_fshandle;
        }

      /* Seek to the previous entry to be replaced */

      pos = net_seekroute_ipv6(&fshandle, index - 1);
      if (pos < 0)
        {
          nerr("ERROR: net_readroute_ipv6 failed: %ld\n", (long)pos);
          ret =(int)pos;
          goto errout_with_fshandle;
        }

      /* Now write the record to its new location */

      nwritten = net_writeroute_ipv6(&fshandle, &route);
      if (nwritten < 0)
        {
          nerr("ERROR: net_readroute_ipv6 failed: %ld\n", (long)nwritten);
          ret = (int)nwritten;
          goto errout_with_fshandle;
        }
    }

  /* Now truncate the one duplicate entry at the end of the file.  This may
   * result in a zero length file.
   */

  filesize = (nentries - 1) * sizeof(struct net_route_ipv6_s);
  ret = file_truncate(&fshandle, filesize);

errout_with_fshandle:
  (void)net_closeroute_ipv6(&fshandle);

errout_with_lock:
  (void)net_unlockroute_ipv6();
  return ret;
}
#endif

#endif /* CONFIG_ROUTE_IPv4_FILEROUTE || CONFIG_ROUTE_IPv6_FILEROUTE  */
