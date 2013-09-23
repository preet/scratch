/*
 * btserver.c
 * This file is part of Android Simple Scrolling
 *
 * Copyright (C) 2011 - Manuel Di Cerbo, Nexus-Computing GmbH
 *
 * Android Simple Scrolling is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Android Simple Scrolling is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Android Simple Scrolling; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, 
 * Boston, MA  02110-1301  USA
 */
 
 /* huge thanks goes to Albert Huang for the Bluez Programming Guide found at
  * http://people.csail.mit.edu/albert/bluez-intro/x604.html
  */
#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <bluetooth/sdp.h>
#include <bluetooth/sdp_lib.h>
#include "simplescrolling.h"

#define SVC_CHANNEL 15
#define SVC_NAME "Scrollpad++"
#define SVC_DESC "A simple Android Scrollpad"
#define SVC_PROV "Nexus-Computing GmbH"

//gcc btserver.c btserver.h simplescrolling.c simplescrolling.h -o btserver -lbluetooth -Wall

sdp_session_t *register_service(void);

int main(int argc, char **argv){
	struct sockaddr_rc loc_addr = { 0 }, rem_addr = { 0 };
	char buf[1] = { 0 };
	int s, client, bytes_read;
	socklen_t opt = sizeof(rem_addr);
	sdp_session_t *sdp_session;

	//register the service an acquire the session
	sdp_session = register_service();
	
	for(;;){
		// allocate socket
		s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

		// bind socket to port 1 of the first available 
		// local bluetooth adapter
		loc_addr.rc_family = AF_BLUETOOTH;
		loc_addr.rc_bdaddr = *BDADDR_ANY;
		loc_addr.rc_channel = (uint8_t) SVC_CHANNEL;
	

		bind(s, (struct sockaddr *)&loc_addr, sizeof(loc_addr));

		// put socket into listening mode
		fprintf(stderr, "listening\n");
		listen(s, 1);

		// accept one connection
		fprintf(stderr, "accepting\n");
		client = accept(s, (struct sockaddr *)&rem_addr, &opt);

		ba2str( &rem_addr.rc_bdaddr, buf );
		fprintf(stderr, "accepted connection from %s\n", buf);
		memset(buf, 0, sizeof(buf));

		// read data from the client
	
		initInput();
		bytes_read = 1;
		while(bytes_read > 0){
			bytes_read = read(client, buf, sizeof(buf));
			scroll(buf[0]);
			printf("received %d\n", buf[0]);
		}	
		deinitInput();

		// close connection
		close(client);
		close(s);		
	}
	
	//close sdp session	
	sdp_close(sdp_session);
	return 0;
}



sdp_session_t *register_service(){
	//4e:65:78:75:73:2d:43:6f:6d:70:75:74:69:6e:67 ~ Nexus-Computing :)4e65 7875 732d 436f 6d70 7574 696e 6700
	//uint32_t service_uuid_int[] = {0x4e65,0x7875,0x732d,0x436f,0x6d70,0x7574,0x696e,0x6700};
	uint8_t service_uuid_int[] = {0x07,0x29,0x3d,0xb4,0xa3, 0x23, 0x4e, 0x07,0x8b, 0x8b,0x25,0x0b,0x34,0x0e, 0x42, 0xa4};
	
	uint8_t rfcomm_channel = SVC_CHANNEL;

	uuid_t root_uuid, l2cap_uuid, rfcomm_uuid, svc_uuid;
	sdp_list_t *l2cap_list = 0, 
		*rfcomm_list = 0,
		*root_list = 0,
		*proto_list = 0, 
		*access_proto_list = 0,
		*class_id_list;
		
	sdp_data_t *channel = 0, *psm = 0;

	sdp_record_t *record = sdp_record_alloc();

	// set the general service ID
	sdp_uuid128_create( &svc_uuid, &service_uuid_int );
	sdp_set_service_id( record, svc_uuid );

	// make the service record publicly browsable
	sdp_uuid16_create(&root_uuid, PUBLIC_BROWSE_GROUP);
	root_list = sdp_list_append(0, &root_uuid);
	sdp_set_browse_groups( record, root_list );

	// set l2cap information
	sdp_uuid16_create(&l2cap_uuid, L2CAP_UUID);
	l2cap_list = sdp_list_append( 0, &l2cap_uuid );
	proto_list = sdp_list_append( 0, l2cap_list );

	// set rfcomm information
	sdp_uuid16_create(&rfcomm_uuid, RFCOMM_UUID);
	channel = sdp_data_alloc(SDP_UINT8, &rfcomm_channel);
	rfcomm_list = sdp_list_append( 0, &rfcomm_uuid );
	sdp_list_append( rfcomm_list, channel );
	sdp_list_append( proto_list, rfcomm_list );

	// attach protocol information to service record
	access_proto_list = sdp_list_append( 0, proto_list );
	sdp_set_access_protos( record, access_proto_list );
	
	//service classes IMPORTANT FOR ANDROID!
	class_id_list = sdp_list_append( 0,  &svc_uuid);
	sdp_set_service_classes( record, class_id_list );
	

	// set the name, provider, and description
	sdp_set_info_attr(record, SVC_NAME, SVC_PROV, SVC_DESC);
	
	int err = 0;
	sdp_session_t *session = 0;

	// connect to the local SDP server, register the service record, and 
	// disconnect
	session = sdp_connect( BDADDR_ANY, BDADDR_LOCAL, SDP_RETRY_IF_BUSY );
	err = sdp_record_register(session, record, 0);
	
	if(err != 0)
		perror("Error registring record");
		
	// cleanup
	sdp_data_free( channel );
	sdp_list_free( l2cap_list, 0 );
	sdp_list_free( rfcomm_list, 0 );
	sdp_list_free( root_list, 0 );
	sdp_list_free( access_proto_list, 0 );

	return session;
}
