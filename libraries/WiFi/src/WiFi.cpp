/* WiFi library that uses the Intel Centrino N-135 wireless */
#include "WiFi.h"
#include <trace.h>
extern "C" {
  #include "utility/wl_definitions.h"
  #include "utility/n135.h"
}

#define MY_TRACE_PREFIX "WiFiClass"

#ifndef ARDUINO_WLAN
#define ARDUINO_WLAN "wlan0"
#endif
#define TMP_PATH	"/tmp/tmp.tmp"

// XXX: don't make assumptions about the value of MAX_SOCK_NUM.
int16_t 	WiFiClass::_state[MAX_SOCK_NUM] = { NA_STATE, NA_STATE, NA_STATE, NA_STATE };
uint16_t 	WiFiClass::_server_port[MAX_SOCK_NUM] = { 0, 0, 0, 0 };

WiFiClass::WiFiClass()
{
	// Driver initialization
	init();
}

uint8_t WiFiClass::getEncrBySsid(int count, char * ssid)
{
	for (int i = 0; i < count; i++ ) {
		//trace_debug("checking ssid %s against %s", _networkSsid[i], ssid);
		if (0 == strcmp(ssid, _networkSsid[i])){
			trace_debug("found ssid %s, encr=%d", ssid, getEncrType(_networkEncr[i]));
			return getEncrType(_networkEncr[i]);
		}
	}
	return 0;
}

uint8_t WiFiClass::getEncrType(char * encr)
{
	/* TODO finish this for other security */
	if (0 == strncmp("TKIP", encr, 4))
		return ENC_TYPE_TKIP;
	else if (0 == strncmp("WEP", encr, 3))
		return ENC_TYPE_WEP;
	else if (0 == strncmp("CCMP", encr, 4))
		return ENC_TYPE_CCMP;
	else
		return ENC_TYPE_NONE;
}

IPAddress WiFiClass::getLocalIP()
{
	FILE *fp = NULL;
	char cmd[128];
	uint8_t ipb[4];
	IPAddress ip;
	trace_debug("getLocalIP");
	sprintf(cmd, "ifconfig %s | egrep \"inet addr\" | cut -d: -f2- > %s",
			ARDUINO_WLAN, TMP_PATH);
	system(cmd);
	if (NULL == (fp = fopen(TMP_PATH, "r"))) {
		trace_error("can't open handle to %s", TMP_PATH);
		return ip;
	}
	fscanf(fp, "%s", cmd); /* inet addr */
	fclose(fp);
	trace_debug("my IP=%s", cmd);
	if(isdigit(cmd[0])) {
		sscanf(cmd, "%hhd.%hhd.%hhd.%hhd", &ipb[0], &ipb[1],
				&ipb[2], &ipb[3]);
		ip._sin.sin_addr.s_addr = ( ((uint32_t)ipb[3])<<24 | \
			((uint32_t)ipb[2])<<16 | ((uint32_t)ipb[1])<<8 | ((uint32_t)ipb[0]) );
		trace_debug("returning ip %3d.%3d.%3d.%3d",
			(ip._sin.sin_addr.s_addr&0x000000FF),
			(ip._sin.sin_addr.s_addr&0x0000FF00)>>8,
			(ip._sin.sin_addr.s_addr&0x00FF0000)>>16,
			(ip._sin.sin_addr.s_addr&0xFF000000)>>24);
	} else {
		ip._sin.sin_addr.s_addr = 0;
	}
	return ip;
}

IPAddress WiFiClass::getSubnet()
{
	FILE *fp = NULL;
	char cmd[128];
	uint8_t ipb[4];
	IPAddress ip;
	trace_debug("getSubnet");
	sprintf(cmd, "ifconfig %s | egrep \"inet addr\" | cut -d: -f2- \
		| cut -d: -f2- | cut -d: -f2- > %s", ARDUINO_WLAN, TMP_PATH);
	system(cmd);
	if (NULL == (fp = fopen(TMP_PATH, "r"))) {
		trace_error("can't open handle to %s", TMP_PATH);
		return ip;
	}
	fscanf(fp, "%s", cmd); /* Mask */
	fclose(fp);
	trace_debug("my IP=%s", cmd);

	//sscanf(cmd, "%d.%d.%d.%d", &ipb[0], &ipb[1],
	sscanf(cmd, "%hhd.%hhd.%hhd.%hhd", &ipb[0], &ipb[1],
			&ipb[2], &ipb[3]);
	ip._sin.sin_addr.s_addr = ( ((uint32_t)ipb[3])<<24 | \
		((uint32_t)ipb[2])<<16 | ((uint32_t)ipb[1])<<8 | ((uint32_t)ipb[0]) );
	trace_debug("returning ip %3d.%3d.%3d.%3d",
		(ip._sin.sin_addr.s_addr&0x000000FF),
		(ip._sin.sin_addr.s_addr&0x0000FF00)>>8,
		(ip._sin.sin_addr.s_addr&0x00FF0000)>>16,
		(ip._sin.sin_addr.s_addr&0xFF000000)>>24);
	return ip;
}

void WiFiClass::init()
{
	FILE *fp = NULL;
	char cmd[50];
	trace_debug("init");
	
	/* start as if we have no hw */
	_status = WL_NO_SHIELD;
	sprintf(cmd, "ls /sys/class/net/ | grep %s > %s", ARDUINO_WLAN, TMP_PATH);
	system(cmd);
	if (NULL == (fp = fopen(TMP_PATH, "r"))) {
		trace_error("can't open handle to %s", TMP_PATH);
		return;
	}
	fread(cmd, sizeof(ARDUINO_WLAN), 1, fp);
	fclose(fp);
	trace_debug("wlan0 = %d, cmd is now %s", sizeof(ARDUINO_WLAN), cmd);
	if(0 == (strncmp(cmd, ARDUINO_WLAN, sizeof(ARDUINO_WLAN)-1))) {
		trace_debug("found %s", ARDUINO_WLAN);
		/* important to clear this file otherwise it takes precedence
			when we do ifconfig up for scanning networks */
		sprintf(cmd, "rm /etc/wpa_supplicant.conf");
		system(cmd);
		/* satisfy Arduino sketches that user this lib */
		strcpy(fwVersion, "1.1.0");
		/* ensure we are not already up from previous sketch */
		/* start from a known good */
		sprintf(cmd, "ifconfig %s down", ARDUINO_WLAN);
		system(cmd);
		delay(3000);

		sprintf(cmd, "ifdown %s", ARDUINO_WLAN);
		system(cmd);
		delay(2000);

		/* needs to be up to do a scan */
		trace_debug("ifconfig %s up", ARDUINO_WLAN);
		sprintf(cmd, "ifconfig %s up", ARDUINO_WLAN);
		system(cmd);
		delay(4000);
		macAddress(_bssid);
		_status = WL_IDLE_STATUS;
	} else {
		trace_error("no %s found %s", ARDUINO_WLAN);
	}
}

uint8_t WiFiClass::getSocket()
{
	trace_debug("getSocket");
	for (uint8_t i = 0; i < MAX_SOCK_NUM; ++i)
	{
		if (WiFiClass::_server_port[i] == 0)
		{
			return i;
		}
	}
	return NO_SOCKET_AVAIL;
}

char* WiFiClass::firmwareVersion()
{
	trace_debug("firmwareVersion");
	return fwVersion;
}

int WiFiClass::begin(char* ssid)
{
	char cmd[256];
	trace_debug("begin ssid=%s",ssid);
	if(ssid == NULL)
		return WL_NO_SSID_AVAIL;

	sprintf(cmd, "iwconfig %s mode managed", ARDUINO_WLAN);
	system(cmd);
	sprintf(cmd, "iwconfig %s essid %s", ARDUINO_WLAN, ssid);
	system(cmd);
	sprintf(cmd, "ifup %s", ARDUINO_WLAN);
	system(cmd);
	delay(5000);
	_local_ip = getLocalIP();
	if(_local_ip._sin.sin_addr.s_addr == 0) {
		trace_debug("didnt get an IP address, cant connect");
		sprintf(cmd, "ifdown %s", ARDUINO_WLAN);
		_status = WL_IDLE_STATUS;
		return _status;
	}
	_subnet = getSubnet();
	strcpy(_ssid, ssid);
	_status = WL_CONNECTED;
	return _status;
}

int WiFiClass::begin(char* ssid, uint8_t key_idx, const char *key)
{
	char cmd[256];
	trace_debug("begin ssid=%s,key_idx=%d,key=%s",ssid,key_idx,key);
	/*
	iwconfig [interface] mode managed key [WEP key] 
	(128 bit WEP use 26 hex characters, 64 bit WEP uses 10) 
	*/
	if(ssid == NULL)
		return WL_NO_SSID_AVAIL;
	if(key == NULL)
		return WL_CONNECT_FAILED;
	sprintf(cmd, "iwconfig wlan0 mode managed key %s", key);
	system(cmd);
	sprintf(cmd, "iwconfig wlan0 essid %s", ssid);
	system(cmd);
	sprintf(cmd, "ifup %s", ARDUINO_WLAN);
	system(cmd);
	delay(5000);
	_local_ip = getLocalIP();
	if(_local_ip._sin.sin_addr.s_addr == 0) {
		trace_debug("didnt get an IP address, cant connect");
		sprintf(cmd, "ifdown %s", ARDUINO_WLAN);
		_status = WL_IDLE_STATUS;
		return _status;
	}
	_subnet = getSubnet();
	strcpy(_ssid, ssid);
	_status = WL_CONNECTED;
	return _status;
}

int WiFiClass::begin(char* ssid, const char *passphrase)
{
	char cmd[256];
	trace_debug("begin ssid=%s,passphrase=%s",ssid,passphrase);
	if(ssid == NULL)
		return WL_NO_SSID_AVAIL;
	if(passphrase == NULL)
		return WL_CONNECT_FAILED;
	sprintf(cmd, "iwconfig wlan0 mode managed");
	system(cmd);
	sprintf(cmd, "iwconfig wlan0 essid %s", ssid);
	system(cmd);
	sprintf(cmd, "wpa_passphrase %s %s  > /etc/wpa_supplicant.conf",
			ssid, passphrase);
	system(cmd);
	sprintf(cmd, "ifup %s", ARDUINO_WLAN);
	system(cmd);
	delay(5000);
	_local_ip = getLocalIP();
	if(_local_ip._sin.sin_addr.s_addr == 0) {
		trace_debug("didnt get an IP address, cant connect");
		sprintf(cmd, "ifdown %s", ARDUINO_WLAN);
		_status = WL_IDLE_STATUS;
		return _status;
	}
	_subnet = getSubnet();
	strcpy(_ssid, ssid);
	_status = WL_CONNECTED;
	return _status;
}

void WiFiClass::config(IPAddress local_ip)
{
	_local_ip = local_ip;
	trace_debug("config local ip %3d.%3d.%3d.%3d",
		(_local_ip._sin.sin_addr.s_addr&0x000000FF),
		(_local_ip._sin.sin_addr.s_addr&0x0000FF00)>>8,
		(_local_ip._sin.sin_addr.s_addr&0x00FF0000)>>16,
		(_local_ip._sin.sin_addr.s_addr&0xFF000000)>>24);
}

void WiFiClass::config(IPAddress local_ip, IPAddress dns_server)
{
	_local_ip = local_ip;
	_dns_server = dns_server;

	trace_debug("config  local ip %3d.%3d.%3d.%3d",
		(_local_ip._sin.sin_addr.s_addr&0x000000FF),
		(_local_ip._sin.sin_addr.s_addr&0x0000FF00)>>8,
		(_local_ip._sin.sin_addr.s_addr&0x00FF0000)>>16,
		(_local_ip._sin.sin_addr.s_addr&0xFF000000)>>24);

	trace_debug("_dns_server ip %3d.%3d.%3d.%3d",
		(_dns_server._sin.sin_addr.s_addr&0x000000FF),
		(_dns_server._sin.sin_addr.s_addr&0x0000FF00)>>8,
		(_dns_server._sin.sin_addr.s_addr&0x00FF0000)>>16,
		(_dns_server._sin.sin_addr.s_addr&0xFF000000)>>24	);
}

void WiFiClass::config(IPAddress local_ip, IPAddress dns_server,
						IPAddress gateway)
{
	_local_ip = local_ip;
	_dns_server = dns_server;
	_gateway = gateway;

	trace_debug("config  local ip %3d.%3d.%3d.%3d",
		(_local_ip._sin.sin_addr.s_addr&0x000000FF),
		(_local_ip._sin.sin_addr.s_addr&0x0000FF00)>>8,
		(_local_ip._sin.sin_addr.s_addr&0x00FF0000)>>16,
		(_local_ip._sin.sin_addr.s_addr&0xFF000000)>>24);

	trace_debug("_dns_server ip %3d.%3d.%3d.%3d",
		(_dns_server._sin.sin_addr.s_addr&0x000000FF),
		(_dns_server._sin.sin_addr.s_addr&0x0000FF00)>>8,
		(_dns_server._sin.sin_addr.s_addr&0x00FF0000)>>16,
		(_dns_server._sin.sin_addr.s_addr&0xFF000000)>>24	);

	trace_debug("_gateway ip %3d.%3d.%3d.%3d",
		(_gateway._sin.sin_addr.s_addr&0x000000FF),
		(_gateway._sin.sin_addr.s_addr&0x0000FF00)>>8,
		(_gateway._sin.sin_addr.s_addr&0x00FF0000)>>16,
		(_gateway._sin.sin_addr.s_addr&0xFF000000)>>24	);
}

void WiFiClass::config(IPAddress local_ip, IPAddress dns_server, 
						IPAddress gateway, IPAddress subnet)
{
	_local_ip = local_ip;
	_dns_server = dns_server;
	_gateway = gateway;
	_subnet = subnet;

	trace_debug("config local ip %3d.%3d.%3d.%3d",
		(_local_ip._sin.sin_addr.s_addr&0x000000FF),
		(_local_ip._sin.sin_addr.s_addr&0x0000FF00)>>8,
		(_local_ip._sin.sin_addr.s_addr&0x00FF0000)>>16,
		(_local_ip._sin.sin_addr.s_addr&0xFF000000)>>24);

	trace_debug("_dns_server ip %3d.%3d.%3d.%3d",
		(_dns_server._sin.sin_addr.s_addr&0x000000FF),
		(_dns_server._sin.sin_addr.s_addr&0x0000FF00)>>8,
		(_dns_server._sin.sin_addr.s_addr&0x00FF0000)>>16,
		(_dns_server._sin.sin_addr.s_addr&0xFF000000)>>24	);

	trace_debug("_gateway ip %3d.%3d.%3d.%3d",
		(_gateway._sin.sin_addr.s_addr&0x000000FF),
		(_gateway._sin.sin_addr.s_addr&0x0000FF00)>>8,
		(_gateway._sin.sin_addr.s_addr&0x00FF0000)>>16,
		(_gateway._sin.sin_addr.s_addr&0xFF000000)>>24	);

	trace_debug("_subnet ip %3d.%3d.%3d.%3d",
		(_subnet._sin.sin_addr.s_addr&0x000000FF),
		(_subnet._sin.sin_addr.s_addr&0x0000FF00)>>8,
		(_subnet._sin.sin_addr.s_addr&0x00FF0000)>>16,
		(_subnet._sin.sin_addr.s_addr&0xFF000000)>>24);
}

void WiFiClass::setDNS(IPAddress dns_server)
{
	_dns_server = dns_server;

	trace_debug("setDNS _dns_server ip %3d.%3d.%3d.%3d",
		(_dns_server._sin.sin_addr.s_addr&0x000000FF),
		(_dns_server._sin.sin_addr.s_addr&0x0000FF00)>>8,
		(_dns_server._sin.sin_addr.s_addr&0x00FF0000)>>16,
		(_dns_server._sin.sin_addr.s_addr&0xFF000000)>>24	);
}

void WiFiClass::setDNS(IPAddress dns_server1, IPAddress dns_server2)
{
	_dns_server = dns_server1;
	_dns_server2 = dns_server2;

	trace_debug("setDNS _dns_server ip %3d.%3d.%3d.%3d",
		(_dns_server._sin.sin_addr.s_addr&0x000000FF),
		(_dns_server._sin.sin_addr.s_addr&0x0000FF00)>>8,
		(_dns_server._sin.sin_addr.s_addr&0x00FF0000)>>16,
		(_dns_server._sin.sin_addr.s_addr&0xFF000000)>>24	);

	trace_debug("setDNS _dns_server2 ip %3d.%3d.%3d.%3d",
		(_dns_server2._sin.sin_addr.s_addr&0x000000FF),
		(_dns_server2._sin.sin_addr.s_addr&0x0000FF00)>>8,
		(_dns_server2._sin.sin_addr.s_addr&0x00FF0000)>>16,
		(_dns_server2._sin.sin_addr.s_addr&0xFF000000)>>24	);
}

int WiFiClass::disconnect()
{
	char cmd[50];
	trace_debug("disconnect");
	if(_status == WL_CONNECTED) {
		sprintf(cmd, "ifdown %s", ARDUINO_WLAN);
		system(cmd);
		delay(2000);
		_ssid[0] = '\0';
		_status = WL_IDLE_STATUS;
		return WL_DISCONNECTED;
	} else {
		trace_error("not connected to AP");
		return WL_DISCONNECTED;
	}
}

uint8_t* WiFiClass::macAddress(uint8_t* mac)
{
	FILE *fp = NULL;
	char cmd[128];
	//uint8_t mac[6];
	sprintf(cmd, "ifconfig %s | egrep \"HWaddr\" | cut -d: -f2- > %s",
			ARDUINO_WLAN, TMP_PATH);
	system(cmd);
	if (NULL == (fp = fopen(TMP_PATH, "r"))) {
		trace_error("can't open handle to %s", TMP_PATH);
		return NULL;
	}
	fscanf(fp, "%s", cmd); /* Ethernet */
	fscanf(fp, "%s", cmd); /* HWaddr */
	fscanf(fp, "%s", cmd); /* XX:XX:XX:XX:XX:XX */
	fclose(fp);
	trace_debug("my mac=%s", cmd);
	//sscanf(cmd, "%x:%x:%x:%x:%x:%x", &mac[0], &mac[1], 
	sscanf(cmd, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &mac[0], &mac[1],
			&mac[2], &mac[3], &mac[4], &mac[5]);
	return mac;
}

IPAddress WiFiClass::localIP()
{
	return _local_ip;
}

IPAddress WiFiClass::subnetMask()
{
	return _subnet;
}

IPAddress WiFiClass::gatewayIP()
{
	return _gateway;
}

char* WiFiClass::SSID()
{
#if 0
	/* this is how to get the SSID on demand */
	FILE *fp = NULL;
	char cmd[256];
	if (_status == WL_CONNECTED) {
		  sprintf(cmd, "iwgetid %s | cut -d: -f2- | sed \"s/\\\"//g\" > %s",
		  			ARDUINO_WLAN, TMP_PATH);
		system(cmd);
		if (NULL == (fp = fopen(TMP_PATH, "r"))) {
			trace_error("can't open handle to %s", TMP_PATH);
			return NULL;
		}
		fseek(fp,0,SEEK_END);
		int size = ftell(fp);
		trace_debug("size is %d", size);
		rewind (fp);
		fread(cmd, size, 1, fp);
		cmd[size] = '\0';
		fclose(fp);
		trace_debug("returning SSID %s", cmd);
		return cmd;
	} else {
		return NULL;
	}
#endif
	return _ssid;
}

uint8_t* WiFiClass::BSSID(uint8_t* bssid)
{
	/* TODO API not clear - are we ok returning ptr we get in */
	trace_debug("BSSID");
	if (_status == WL_CONNECTED) {
		FILE *fp = NULL;
		char cmd[256];
		//uint8_t mac[6];
		sprintf(cmd, "iwconfig %s | egrep \" Point\" | cut -d: -f2- \
					| cut -d: -f2- | cut -d: -f2- > %s",
					ARDUINO_WLAN, TMP_PATH);
		system(cmd);
		if (NULL == (fp = fopen(TMP_PATH, "r"))) {
			trace_error("can't open handle to %s", TMP_PATH);
			return NULL;
		}
		fscanf(fp, "%s", cmd);
		fclose(fp);
		trace_debug("BSSID=%s", cmd);
		//sscanf(cmd, "%x:%x:%x:%x:%x:%x", &bssid[0], &bssid[1],
		sscanf(cmd, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &bssid[0], &bssid[1],
					&bssid[2], &bssid[3], &bssid[4], &bssid[5]);
		return bssid;
	} else {
		trace_error("%s not connected", ARDUINO_WLAN);
		return NULL;
	}
}

int32_t WiFiClass::RSSI()
{
	trace_debug("RSSI");
	if (_status == WL_CONNECTED) {
		FILE *fp = NULL;
		char cmd[256];
		sprintf(cmd, "iwconfig %s | egrep \"Signal level\" \
					| cut -d- -f2- > %s", ARDUINO_WLAN, TMP_PATH);
		system(cmd);
		if (NULL == (fp = fopen(TMP_PATH, "r"))) {
			trace_error("can't open handle to %s", TMP_PATH);
			return 0;
		}
		fscanf(fp, "%s", cmd);
		fclose(fp);
		trace_debug("RSSI=%s", cmd);
		return (0 - atoi(cmd)) ;
	} else {
		trace_error("%s not connected", ARDUINO_WLAN);
		return 0;
	}
}

uint8_t WiFiClass::encryptionType()
{
	int count;
	if (_status == WL_CONNECTED) {
		/* no easy way to do this apart from scanning again */
		trace_debug("doing scan to get encryption");
		count = scanNetworks();
		return getEncrBySsid(count, _ssid);
	} else {
		trace_error("%s not connected", ARDUINO_WLAN);
		return 0;
	}
}

int8_t WiFiClass::scanNetworks()
{
	FILE *fp = NULL;
	char cmd[1024];
	char str [512];
	sprintf(cmd, "iwlist %s scan | egrep \" Group Cipher|Quality|ESSID\" \
			| cut -d: -f2- | cut -d= -f2- | cut -d= -f2- | sed \"s/\\\"//g\" > %s",
			ARDUINO_WLAN, TMP_PATH);
	system(cmd);

	if (NULL == (fp = fopen(TMP_PATH, "r"))) {
		trace_error("can't open handle to %s", TMP_PATH);
		return 0;
	}
	fseek(fp,0,SEEK_END);
	int size = ftell(fp);
	trace_debug("size is %d", size);
	rewind (fp);
	int count=0;
	while (fscanf(fp, "%s", str)!=EOF) {
		if(str[0] == '-') {
			_networkRssi[count] = atoi(str);
			fscanf(fp, "%s", str); fscanf(fp, "%s", str);
			strcpy(_networkSsid[count], str);
			fscanf(fp, "%s", str);
			strcpy(_networkEncr[count], str);
			//_networkEncr[count] = getEncrType(str);
			count++;
		}
	}
#ifdef __DEBUG
	trace_debug("found %d networks in range", count);
	for(int i=0; i < count; i++) {
		trace_debug("%d. ssid=%s, rssi=%d, , encr=%s",
			i, _networkSsid[i], _networkRssi[i], _networkEncr[i]);
	}
#endif
	fclose(fp);
	return count;
}

char* WiFiClass::SSID(uint8_t networkItem)
{
#if 0
	FILE *fp = NULL;
	char cmd[128];
	char str[128];
	sprintf(cmd, "iwlist %s scan | egrep \" ESSID\" | cut -d: -f2- > %s", ARDUINO_WLAN, TMP_PATH);
	system(cmd);
	if (NULL == (fp = fopen(TMP_PATH, "r"))) {
		trace_error("can't open handle to %s", TMP_PATH);
		return;
	}
	if (fp) {
		while (fscanf(fp, "%s", str)!=EOF)
			trace_debug("str = %s",str);
		fclose(fp);
	}
#endif        
	return _networkSsid[networkItem];
}

int32_t WiFiClass::RSSI(uint8_t networkItem)
{
	if(networkItem >= WL_NETWORKS_LIST_MAXNUM)
		return 0;
	else
		return _networkRssi[networkItem];
}

uint8_t WiFiClass::encryptionType(uint8_t networkItem)
{
	if(networkItem >= WL_NETWORKS_LIST_MAXNUM)
		return 0;
	else
		return getEncrType(_networkEncr[networkItem]);
}

uint8_t WiFiClass::status()
{
    return _status;
}

int WiFiClass::hostByName(const char* aHostname, IPAddress& aResult)
{
	//return WiFiDrv::getHostByName(aHostname, aResult);
	return 0;
}

WiFiClass WiFi;
