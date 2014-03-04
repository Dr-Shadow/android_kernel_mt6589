/***********************************************************
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** VENDOR_EDIT
** File: - networkanalyser.c
** Description: permissions intercept policy, add for permission intercept.
** Version: 1.0
** Date : 2013/04/18	
** Author: wangy@OnLineRD.Framework.pms
** 
** ------------------------------- Revision History: -------------------------------
**  	<author>		<data> 	   <version >	       <desc>
**       wangy       2013/04/18      1.0          build this moudle
****************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/skbuff.h>
#include <linux/ip.h>
#include <net/ip.h>
#include <net/sock.h>
#include <linux/slab.h>

#include <linux/socket.h>
#include <linux/netlink.h>

#define NETLINK_PERMISSON_TYPE 30
#define MAX_MSGSIZE 60
#define MSG_TYPE_PID 1
#define MSG_TYPE_RESET 2
#define DEBUG_ENABLE

static char uidArray[10000];
static struct nf_hook_ops nfho;
struct sock *per_sock;
static int clientPid = -1;

void msg_receive(struct sk_buff *skb) {
	int uid = -1;
	int index = 0;
	struct nlmsghdr *nlh = NULL;
	char *data = NULL;

	#ifdef DEBUG_ENABLE
	printk(KERN_ERR "msg_receive");
	#endif

	if (!skb) {
		printk(KERN_ERR "msg_receive, skb = null");
		return;
	}
	
	nlh = (struct nlmsghdr *)skb->data;
	data = (char*) NLMSG_DATA(nlh);
	if (MSG_TYPE_PID == nlh->nlmsg_type) {
		clientPid = nlh->nlmsg_pid;
		memset(uidArray, 0, sizeof(uidArray));
		#ifdef DEBUG_ENABLE 
		printk(KERN_ERR "msg_receive, clientPid=%d", clientPid);
		#endif
	} else if(MSG_TYPE_RESET == nlh->nlmsg_type) {
		uid = (int) simple_strtoull(data, NULL, 10);
		index = uid - 10000;
		#ifdef DEBUG_ENABLE
		printk(KERN_ERR "msg_receive, index=%d", index);
		#endif
		if (index >= 0 && index < 9999) {
			uidArray[index] = 0;
		}
		#ifdef DEBUG_ENABLE
		printk(KERN_ERR "msg_receive, reset uid=%d", uid);
		#endif
	}

	//wake_up_interruptible(sk->sk_sleep);
}

void send_network_msg(int uid) {
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	char uidStr[10];
	int ret;
	#ifdef DEBUG_ENABLE
	printk(KERN_ERR "send_network_msg, uid = %d", uid);
	#endif
	snprintf(uidStr, 10, "%d", uid);
	if(per_sock) {
		skb = alloc_skb(NLMSG_SPACE(MAX_MSGSIZE), GFP_ATOMIC);
		nlh = NLMSG_PUT(skb, 0, 0, 0, NLMSG_SPACE(MAX_MSGSIZE));
		strcpy((char*)NLMSG_DATA(nlh), uidStr);
		nlh->nlmsg_len = NLMSG_SPACE(MAX_MSGSIZE);
		nlh->nlmsg_pid = 0;
		nlh->nlmsg_flags = 0;
		
		NETLINK_CB(skb).pid = 0;
		NETLINK_CB(skb).dst_group = 0;
		ret = netlink_unicast(per_sock, skb, clientPid, MSG_DONTWAIT);
		#ifdef DEBUG_ENABLE
		printk(KERN_ERR "netlink_unicast, ret=%d", ret);
		#endif
		return;
	nlmsg_failure:
		printk(KERN_ERR "netlink_unicast error!");
	    if (skb) {
		    kfree_skb(skb);
		}
	}
}

void create_permission_socket(void) {
	per_sock = netlink_kernel_create(&init_net, NETLINK_PERMISSON_TYPE, 0, msg_receive, NULL, THIS_MODULE);
	if (!per_sock) {
		printk(KERN_ERR "can not create a netlink socket.");
	}
}

unsigned int hook_func(unsigned int hooknum,
                       struct sk_buff *skb,
                       const struct net_device *in,
                       const struct net_device *out,
                       int (*okfn)(struct sk_buff *)) {

	int uid = -1;
	int index = 0;

	if (!skb->sk) {
		 printk(KERN_ERR "hook_func skb->sk = NULL");
		 return NF_ACCEPT; 
	} else if(!skb->sk->sk_socket) {
		printk(KERN_ERR "hook_func skb->sk->sk_socket = NULL");
		return NF_ACCEPT; 
	} else {
		struct inode *inode = SOCK_INODE(skb->sk->sk_socket);
		if (inode) {
			uid = inode->i_uid;
			if (10000 > uid) {
			    return NF_ACCEPT;
			}
			#ifdef DEBUG_ENABLE
			printk(KERN_ERR "hook_func, -----inode->i_uid = %d---------", uid);
			#endif
			if (clientPid <= 0 ) {
				printk(KERN_ERR "hook_func, clientPid=%d", clientPid);
				return NF_ACCEPT;
		    }
			index = uid - 10000;
			if (index >= 0 && index <= 9999 && uidArray[index] == 0) {
				uidArray[index] = 1;
				send_network_msg(uid);
			}
		}
	}
    return NF_ACCEPT; 
}

static int __init permission_intercept_init(void) {
    nfho.hook = hook_func;        
    nfho.hooknum  = NF_INET_LOCAL_OUT;
    nfho.pf       = PF_INET;
    nfho.priority = NF_IP_PRI_RAW;
    nf_register_hook(&nfho);
	create_permission_socket();
    return 0;
}

module_init(permission_intercept_init);

static void __exit permission_intercept_exit(void) {
	nf_unregister_hook(&nfho);
	if(per_sock) {
		sock_release(per_sock->sk_socket);
	}
}
module_exit(permission_intercept_exit);

MODULE_AUTHOR("eagle");
MODULE_DESCRIPTION("Permission intercept driver");
MODULE_LICENSE("GPL");