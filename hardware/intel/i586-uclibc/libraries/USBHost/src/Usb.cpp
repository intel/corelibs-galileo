/* Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

Contact information
-------------------

Circuits At Home, LTD
Web      :  http://www.circuitsathome.com
e-mail   :  support@circuitsathome.com
*/

/* USB functions */

#include "Arduino.h"
#include "Usb.h"
#include <variant.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/types.h>

#include <trace.h>

//static uint32_t usb_error = 0;
static uint32_t usb_task_state = USB_DETACHED_SUBSTATE_INITIALIZE;

#define MY_TRACE_PREFIX "usbhost"
#define test_bit(bit, array)  (array [bit / 8] & (1 << (bit % 8)))

/**
 * \brief USBHost class constructor.
 */
USBHost::USBHost() : bmHubPre(0)
{
	DIR * pdir = NULL;
	int fd = -1, i = 0;
	struct dirent* direntp = NULL;
	char buf[0x40];
	const char * input_path = "/dev/input";
	uint8_t evtype_bitmask[(EV_MAX + 7) / 8];
	input_count = ctReportParsers = 0;

	//for (i = 0; i < USB_NUMDEVICES; ++i)
	//	devParsers[i] = NULL;
	//}

	// Init linked list
	phead = NULL;

	// Set up state machine
	usb_task_state = USB_DETACHED_SUBSTATE_INITIALIZE;

	// Init host stack
	init();

	pdir = opendir(input_path);
	if (pdir == NULL){
	    fprintf(stderr,"unable to open /dev/input directory !");
	    fflush(stdout);
	    exit (1);
	}

	for(;;) {
		direntp = readdir( pdir );
		if( direntp == NULL ) break;

		trace_info( "%s", direntp->d_name );
		if(!strcmp(direntp->d_name, ".") || !strcmp(direntp->d_name, "..")){
		    trace_debug("Skipping dir entry %s\n", direntp->d_name);
		    continue;
		}

		memset(buf, 0x00, sizeof(buf));
		snprintf(buf, sizeof(buf),"%s/%s", input_path, direntp->d_name);


		fd = open(buf, O_RDWR);
		if (fd < 0){
			close(fd);
			trace_error( "unable to open %s O_RDWR\n", buf);
			sleep(10);
			exit(-1);
		}

		if (ioctl (fd, EVIOCGNAME (sizeof (buf)), buf) < 0){
			trace_error( "EVIOCGNAME ioctl error");
			sleep(10);
			exit (1);
		}

		trace_debug ("found \"%s\"", buf);

		if (ioctl (fd, EVIOCGBIT (0, sizeof (evtype_bitmask)), evtype_bitmask) < 0) {
			trace_debug("EVIOCGBIT ioctl failed");
			sleep(10);
			exit (1);
		}

		trace_info ("Supported event types:");
		for (i = 0; i < EV_MAX; i++) {
			if (test_bit (i, evtype_bitmask)) {

				/* this means that the bit is set in the event types list */
				trace_info("  Event type 0x%02x ", i);
				switch (i) {
					case EV_SYN :
						trace_info(" (Sync?)\n");
						break;
					case EV_KEY :
						trace_info(" (Keys or Buttons)\n");
						break;
					case EV_REL :
						trace_info(" (Relative Axes)\n");
						break;
					case EV_ABS :
						trace_info(" (Absolute Axes)\n");
						break;
					case EV_LED :
						trace_info(" (LEDs)\n");
						break;
					case EV_REP :
						trace_info(" (Repeat)\n");
						break;
					default:
						trace_error(" (Unknown event type)\n");
				}
			}
		}

		if ( addUSBDevEntry(fd, evtype_bitmask) != 0){
			trace_error("unable to add new entry to input event linked list !");
			sleep(10);
			exit(-1);
		}
	}

        closedir(pdir);
}

/**
 * \brief Add input device to list of input device - not really USB at all....
 */
int USBHost::addUSBDevEntry(int ihandle, uint8_t * evtype_bitmask)
{
	int ret = 0;
	struct usb_dev * pusb_dev = NULL;

	/* add new entry to end of linked list */
	pusb_dev = (struct usb_dev*)malloc(sizeof(struct usb_dev));
	if (pusb_dev == NULL){
		trace_error("%s oom malloc %d bytes fail!", __func__, sizeof(struct usb_dev));
		ret = -ENOMEM;
		goto done;
	}

	pusb_dev->ihandle = ihandle;
	memcpy(&pusb_dev->evtype_bitmask, evtype_bitmask, sizeof(pusb_dev->evtype_bitmask));
	pusb_dev->pnext = NULL;
	pusb_dev->pprev = NULL;

	if (phead == NULL){
		phead = pusb_dev;
	}else{
		pusb_dev->pprev = phead;

		if(phead->pnext != NULL){
			pusb_dev->pnext = phead->pnext;
			pusb_dev->pnext->pprev = pusb_dev;
		}
		phead->pnext = pusb_dev;
	}
	input_count++;
done:
	return ret;

}
/**
 * \brief Initialize USBHost class.
 */
void USBHost::init()
{
	devConfigIndex	= 0;
	bmHubPre	= 0;
}


/**
 * \brief Get USBHost state.
 *
 * \return USB enumeration status (see USBHost::task).
 */
uint32_t USBHost::getUsbTaskState(void)
{
    return (usb_task_state);
}

/**
 * \brief Set USB state.
 *
 * \param state New USBHost status to be set.
 */
void USBHost::setUsbTaskState(uint32_t state)
{
    usb_task_state = state;
}

/**
 * \brief Get endpoint info from USB device address and device endpoint.
 *
 * \note This function should be used to know which host pipe is being used for
 * the corresponding device endpoint.
 *
 * \param addr USB device address.
 * \param ep USB device endpoint number.
 *
 * \return Pointer to an EpInfo structure.
 */
EpInfo* USBHost::getEpInfoEntry(uint32_t addr, uint32_t ep)
{
	#if 0
	UsbDevice *p = addrPool.GetUsbDevicePtr(addr);

	if (!p || !p->epinfo)
		return NULL;

	EpInfo *pep = p->epinfo;

	for (uint32_t i = 0; i < p->epcount; i++)
	{
		if (pep->deviceEpNum == ep)
			return pep;

		pep++;
	}
	#endif
	return NULL;
}

/**
 * \brief Set device endpoint entry.
 *
 * \note Each device is different and has a different number of endpoints.
 * This function sets endpoint record structure to the device using address
 * addr in the address pool.
 *
 * \param ul_pipe Pipe address.
 * \param ul_token_type Token type.
 *
 * \retval 0 on success.
 * \retval USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL device not found.
 */
uint32_t USBHost::setEpInfoEntry(uint32_t addr, uint32_t epcount, EpInfo* eprecord_ptr)
{
	#if 0
	if (!eprecord_ptr)
		return USB_ERROR_INVALID_ARGUMENT;

	UsbDevice *p = addrPool.GetUsbDevicePtr(addr);

	if (!p)
		return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;

	p->address	= addr;
	p->epinfo	= eprecord_ptr;
	p->epcount	= epcount;
	#endif

	return 0;
}

/**
 * \brief Set host pipe target address and set ppep pointer to the endpoint
 * structure matching the specified USB device address and endpoint.
 *
 * \param addr USB device address.
 * \param ep USB device endpoint number.
 * \param ppep Endpoint info structure pointer set by setPipeAddress.
 * \param nak_limit Maximum number of NAK permitted.
 *
 * \retval 0 on success.
 * \retval USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL device not found.
 * \retval USB_ERROR_EPINFO_IS_NULL no endpoint structure found for this device.
 * \retval USB_ERROR_EP_NOT_FOUND_IN_TBL the specified device endpoint cannot be found.
 */
uint32_t USBHost::setPipeAddress(uint32_t addr, uint32_t ep, EpInfo **ppep, uint32_t &nak_limit)
{
	#if 0
	UsbDevice *p = addrPool.GetUsbDevicePtr(addr);

	if (!p)
		return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;

 	if (!p->epinfo)
		return USB_ERROR_EPINFO_IS_NULL;

	*ppep = getEpInfoEntry(addr, ep);

	if (!*ppep)
		return USB_ERROR_EP_NOT_FOUND_IN_TBL;

	nak_limit = (0x0001UL << (((*ppep)->bmNakPower > USB_NAK_MAX_POWER ) ? USB_NAK_MAX_POWER : (*ppep)->bmNakPower));
	nak_limit--;

	// Set peripheral address
	TRACE_USBHOST(printf("     => SetAddress deviceEP=%lu configued as hostPIPE=%lu sending to address=%lu\r\n", ep, (*ppep)->hostPipeNum, addr);)
	uhd_configure_address((*ppep)->hostPipeNum, addr);
	#endif

	return 0;
}

/**
 * \brief Send a control request.
 * Sets address, endpoint, fills control packet with necessary data, dispatches
 * control packet, and initiates bulk IN transfer depending on request.
 *
 * \param addr USB device address.
 * \param ep USB device endpoint number.
 * \param bmReqType Request direction.
 * \param bRequest Request type.
 * \param wValLo Value low.
 * \param wValHi Value high.
 * \param wInd Index field.
 * \param total Request length.
 * \param nbytes Number of bytes to read.
 * \param dataptr Data pointer.
 * \param p USB class reader.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::ctrlReq(uint32_t addr, uint32_t ep, uint8_t bmReqType, uint8_t bRequest, uint8_t wValLo, uint8_t wValHi, uint16_t wInd, uint16_t total, uint32_t nbytes, uint8_t* dataptr, USBReadParser *p)
{
	#if 0
	// Request direction, IN or OUT
	uint32_t direction = 0;
	uint32_t rcode = 0;
	SETUP_PKT setup_pkt;

	EpInfo *pep = 0;
	uint32_t nak_limit;

	TRACE_USBHOST(printf("    => ctrlReq\r\n");)

	// Set peripheral address
	rcode = setPipeAddress(addr, ep, &pep, nak_limit);
	if (rcode)
		return rcode;

	// Allocate Pipe0 with default 64 bytes size if not already initialized
	// TODO : perform a get device descriptor first to get device endpoint size (else data can be missed if device ep0 > host pipe0)
	rcode = UHD_Pipe0_Alloc(0, 64);
	if (rcode)
	{
		TRACE_USBHOST(printf("/!\\ USBHost::ctrlReq : EP0 allocation error: %lu\r\n", rcode);)
		return (rcode);
	}

	// Determine request direction
	direction = ((bmReqType & 0x80 ) > 0);

	// Fill in setup packet
    setup_pkt.ReqType_u.bmRequestType	= bmReqType;
    setup_pkt.bRequest					= bRequest;
    setup_pkt.wVal_u.wValueLo			= wValLo;
    setup_pkt.wVal_u.wValueHi			= wValHi;
    setup_pkt.wIndex					= wInd;
    setup_pkt.wLength					= total;

	// Configure and write the setup packet into the FIFO
	uhd_configure_pipe_token(0, tokSETUP);
	UHD_Pipe_Write(pep->hostPipeNum, 8, (uint8_t *)&setup_pkt);

	// Dispatch packet
	rcode = dispatchPkt(tokSETUP, pep->hostPipeNum, nak_limit);
	if (rcode)
	{
		// Return HRSLT if not zero
		TRACE_USBHOST(printf("/!\\ USBHost::ctrlReq : Setup packet error: %lu\r\n", rcode);)
		return (rcode);
	}

	// Data stage (if present)
	if (dataptr != 0)
	{
		if (direction)
		{
			// IN transfer
			TRACE_USBHOST(printf("    => ctrlData IN\r\n");)
			uint32_t left = total;

			while (left)
			{
				// Bytes read into buffer
				uint32_t read = nbytes;

				rcode = InTransfer(pep, nak_limit, &read, dataptr);
				if (rcode)
					return rcode;

				// Invoke callback function if inTransfer completed successfuly and callback function pointer is specified
				if (!rcode && p)
					((USBReadParser*)p)->Parse(read, dataptr, total - left);

				left -= read;

				if (read < nbytes)
					break;
			}
		}
		else
		{
			// OUT transfer
			TRACE_USBHOST(printf("    => ctrlData OUT\r\n");)
			rcode = OutTransfer(pep, nak_limit, nbytes, dataptr);
		}

		if (rcode)
		{
			TRACE_USBHOST(printf("/!\\ USBHost::ctrlData : Data packet error: %lu\r\n", rcode);)
			return (rcode);
		}
	}

	// Status stage
	return dispatchPkt((direction) ? tokOUTHS : tokINHS, pep->hostPipeNum, nak_limit);
	#endif

	return 0;
}

/**
 * \brief Perform IN request to the specified USB device.
 *
 * \note This function handles multiple packets (if necessary) and can
 * receive a maximum of 'nbytesptr' bytes. It keep sending INs and writes data
 * to memory area pointed by 'data'. The actual amount of received bytes is
 * stored in 'nbytesptr'.
 *
 * \param addr USB device address.
 * \param ep USB device endpoint number.
 * \param nbytesptr Receive buffer size. It is set to the amount of received
 * bytes when the function returns.
 * \param data Buffer to store received data.
 *
 * \return 0 on success, error code otherwise.
 */
 uint32_t USBHost::inTransfer(uint32_t addr, uint32_t ep, uint32_t *nbytesptr, uint8_t* data)
{
	#if 0
	EpInfo *pep = NULL;
	uint32_t nak_limit = 0;

	uint32_t rcode = setPipeAddress(addr, ep, &pep, nak_limit);

	if (rcode)
	{
		return rcode;
	}

	return InTransfer(pep, nak_limit, nbytesptr, data);
	#endif

	return 0;
}

uint32_t USBHost::InTransfer(EpInfo *pep, uint32_t nak_limit, uint32_t *nbytesptr, uint8_t* data)
{
	#if 0
	uint32_t rcode = 0;
	uint32_t pktsize = 0;
	uint32_t nbytes = *nbytesptr;
	uint32_t maxpktsize = pep->maxPktSize;

	*nbytesptr = 0;

	while (1)
	{
		// Use a 'return' to exit this loop
		// IN packet to EP-'endpoint'. Function takes care of NAKS.
        rcode = dispatchPkt(tokIN, pep->hostPipeNum, nak_limit);
        if (rcode)
		{
			if (rcode == 1)
			{
				// Pipe freeze is mandatory to avoid sending IN endlessly (else reception becomes messy then)
				uhd_freeze_pipe(pep->hostPipeNum);
			}
			// Should be 1, indicating NAK. Else return error code.
            return rcode;
        }

		// Number of received bytes
		pktsize = uhd_byte_count(pep->hostPipeNum);
		if (nbytes < pktsize)
		{
			TRACE_USBHOST(printf("/!\\ USBHost::InTransfer : receive buffer is too small, size=%lu, expected=%lu\r\n", nbytes, pktsize);)
		}
        data += UHD_Pipe_Read(pep->hostPipeNum, pktsize, data);

		// Add this packet's byte count to total transfer length
        *nbytesptr += pktsize;

        // The transfer is complete under two conditions:
        // 1. The device sent a short packet (L.T. maxPacketSize)
        // 2. 'nbytes' have been transferred.
        if ((pktsize < maxpktsize) || (*nbytesptr >= nbytes))
		{
            return 0;
        }
	}
	#endif
	return 0;
}

/**
 * \brief Perform OUT request to the specified USB device.
 *
 * \note This function handles multiple packets (if necessary) and sends
 * 'nbytes' bytes.
 *
 * \param addr USB device address.
 * \param ep USB device endpoint number.
 * \param nbytes Buffer size to be sent.
 * \param data Buffer to send.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::outTransfer(uint32_t addr, uint32_t ep, uint32_t nbytes, uint8_t* data)
{
	#if 0
	EpInfo *pep = NULL;
	uint32_t nak_limit = 0;

	uint32_t rcode = setPipeAddress(addr, ep, &pep, nak_limit);

	if (rcode)
	{
		return rcode;
	}

	return OutTransfer(pep, nak_limit, nbytes, data);
	#endif

	return 0;
}

uint32_t USBHost::OutTransfer(EpInfo *pep, uint32_t nak_limit, uint32_t nbytes, uint8_t *data)
{
	#if 0
	uint32_t rcode = 0;
	uint32_t bytes_tosend = 0;
	uint32_t bytes_left = nbytes;
	uint32_t maxpktsize = pep->maxPktSize;

	if (maxpktsize < 1)
		return USB_ERROR_INVALID_MAX_PKT_SIZE;

	while (bytes_left)
	{
		bytes_tosend = (bytes_left >= maxpktsize) ? maxpktsize : bytes_left;

		// Write FIFO
		UHD_Pipe_Write(pep->hostPipeNum, bytes_tosend, data);

		// Use a 'return' to exit this loop
		// OUT packet to EP-'endpoint'. Function takes care of NAKS.
		rcode = dispatchPkt(tokOUT, pep->hostPipeNum, nak_limit);
		if (rcode)
		{
			// Should be 0, indicating ACK. Else return error code.
			return rcode;
		}

		bytes_left -= bytes_tosend;
		data += bytes_tosend;
	}

	// Should be 0 in all cases
	return rcode;
	#endif

	return 0;
}

/**
 * \brief Dispatch USB packet.
 *
 * \note Ensure peripheral address is set and relevant buffer is loaded/empty.
 * If NAK, tries to re-send up to nak_limit times.
 * If nak_limit == 0, do not count NAKs, exit after timeout.
 *
 * \param token Token type (Setup, In or Out).
 * \param hostPipeNum Host pipe number to use for sending USB packet.
 * \param nak_limit Maximum number of NAK permitted.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::dispatchPkt(uint32_t token, uint32_t hostPipeNum, uint32_t nak_limit)
{
	#if 0
	uint32_t timeout = millis() + USB_XFER_TIMEOUT;
	uint32_t nak_count = 0;
	uint32_t rcode = USB_ERROR_TRANSFER_TIMEOUT;

	TRACE_USBHOST(printf("     => dispatchPkt token=%lu pipe=%lu nak_limit=%lu\r\n", token, hostPipeNum, nak_limit);)

	// Launch the transfer
	UHD_Pipe_Send(hostPipeNum, token);

	// Check timeout but don't hold timeout if VBUS is lost
	while ((timeout > millis()) && (UHD_GetVBUSState() == UHD_STATE_CONNECTED))
	{
		// Wait for transfer completion
		if (UHD_Pipe_Is_Transfer_Complete(hostPipeNum, token))
		{
			return 0;
		}

		// Is NAK received?
		if (Is_uhd_nak_received(hostPipeNum))
		{
			uhd_ack_nak_received(hostPipeNum);
			nak_count++;

			if (nak_limit && (nak_count == nak_limit))
			{
				// Return NAK
				return 1;
			}
		}
	}

	return rcode;
	#endif
	return 0;
}

/**
 * \brief Configure device using known device classes.
 * The device get a new address even if its class remain unknown.
 *
 * \param parent USB device address of the device's parent (0 if root).
 * \param port USB device base address (see AddressPoolImpl).
 * \param lowspeed Device speed.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::Configuring(uint32_t parent, uint32_t port, uint32_t lowspeed)
{
	#if 0
	uint32_t rcode = 0;

	for (; devConfigIndex < USB_NUMDEVICES; ++devConfigIndex)
	{
		if (!devConfig[devConfigIndex])
			continue;

		rcode = devConfig[devConfigIndex]->Init(parent, port, lowspeed);

		if (!rcode)
		{
			TRACE_USBHOST(printf("USBHost::Configuring : found device class!\r\n");)
			devConfigIndex = 0;
			return 0;
		}


		if (rcode == USB_DEV_CONFIG_ERROR_DEVICE_NOT_SUPPORTED)
		{
			TRACE_USBHOST(printf("USBHost::Configuring : ERROR : device not supported!\r\n");)
		}
		else if (rcode == USB_ERROR_CLASS_INSTANCE_ALREADY_IN_USE)
		{
			TRACE_USBHOST(printf("USBHost::Configuring : ERROR : class instance already in use!\r\n");)
		}
		else
		{
			// in case of an error devConfigIndex should be reset to 0
			// in order to start from the very beginning the next time
			// the program gets here
			if (rcode != USB_DEV_CONFIG_ERROR_DEVICE_INIT_INCOMPLETE)
				devConfigIndex = 0;

			return rcode;
		}
	}

	// Device class is not supported by any of the registered classes
	devConfigIndex = 0;

	rcode = DefaultAddressing(parent, port, lowspeed);

	return rcode;
	#endif

	return 0;
}

/**
 * \brief Configure device with unknown USB class.
 *
 * \param parent USB device address of the device's parent (0 if root).
 * \param port USB device base address (see AddressPoolImpl).
 * \param lowspeed Device speed.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::DefaultAddressing(uint32_t parent, uint32_t port, uint32_t lowspeed)
{
	#if 0
	uint32_t rcode = 0;
	UsbDevice *p0 = 0, *p = 0;

	// Get pointer to pseudo device with address 0 assigned
	p0 = addrPool.GetUsbDevicePtr(0);

	if (!p0)
		return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;

	if (!p0->epinfo)
		return USB_ERROR_EPINFO_IS_NULL;

	p0->lowspeed = (lowspeed) ? 1 : 0;

	// Allocate new address according to device class
	uint32_t bAddress = addrPool.AllocAddress(parent, 0, port);

	if (!bAddress)
		return USB_ERROR_OUT_OF_ADDRESS_SPACE_IN_POOL;

	p = addrPool.GetUsbDevicePtr(bAddress);

	if (!p)
		return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;

	p->lowspeed = lowspeed;

	// Assign new address to the device
	rcode = setAddr(0, 0, bAddress);

	if (rcode)
	{
		TRACE_USBHOST(printf("/!\\ USBHost::DefaultAddressing : Set address failed with code: %lu\r\n", rcode);)
		addrPool.FreeAddress(bAddress);
		bAddress = 0;
		return rcode;
	}
	#endif

	return 0;
}

/**
 * \brief Release device and free associated resources.
 *
 * \param addr USB device address.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::ReleaseDevice(uint32_t addr)
{
	#if 0
	if (!addr)
		return 0;

	for (uint32_t i = 0; i < USB_NUMDEVICES; ++i)
	{
		if (devConfig[i]->GetAddress() == addr)
		{
			return devConfig[i]->Release();
		}
	}
	#endif

	return 0;
}

/**
 * \brief Get device descriptor.
 *
 * \param addr USB device address.
 * \param ep USB device endpoint number.
 * \param nbytes Buffer size.
 * \param dataptr Buffer to store received descriptor.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::getDevDescr(uint32_t addr, uint32_t ep, uint32_t nbytes, uint8_t* dataptr)
{
    #if 0
	return (ctrlReq(addr, ep, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, 0x00, USB_DESCRIPTOR_DEVICE, 0x0000, nbytes, nbytes, dataptr, 0));
    #endif
    return 0;
}

/**
 * \brief Get configuration descriptor.
 *
 * \param addr USB device address.
 * \param ep USB device endpoint number.
 * \param nbytes Buffer size.
 * \param conf Configuration number.
 * \param dataptr Buffer to store received descriptor.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::getConfDescr(uint32_t addr, uint32_t ep, uint32_t nbytes, uint32_t conf, uint8_t* dataptr)
{
	#if 0
	return (ctrlReq(addr, ep, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, conf, USB_DESCRIPTOR_CONFIGURATION, 0x0000, nbytes, nbytes, dataptr, 0));
	#endif
	return 0;
}

/**
 * \brief Get configuration descriptor and extract endpoints using USBReadParser object.
 *
 * \param addr USB device address.
 * \param ep USB device endpoint number.
 * \param conf Configuration number.
 * \param p USBReadParser object pointer used to extract endpoints.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::getConfDescr(uint32_t addr, uint32_t ep, uint32_t conf, USBReadParser *p)
{
	#if 0
	const uint32_t bufSize = 64;
	uint8_t buf[bufSize];

	uint32_t ret = getConfDescr(addr, ep, 8, conf, buf);

	if (ret)
		return ret;

	uint32_t total = ((USB_CONFIGURATION_DESCRIPTOR*)buf)->wTotalLength;
	delay(100);

	return (ctrlReq(addr, ep, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, conf, USB_DESCRIPTOR_CONFIGURATION, 0x0000, total, bufSize, buf, p));
	#endif
	return 0;
}

/**
 * \brief Get string descriptor.
 *
 * \param addr USB device address.
 * \param ep USB device endpoint number.
 * \param nbytes Buffer size.
 * \param index String index.
 * \param langid Language ID.
 * \param dataptr Buffer to store received descriptor.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::getStrDescr(uint32_t addr, uint32_t ep, uint32_t nbytes, uint8_t index, uint16_t langid, uint8_t* dataptr)
{
	#if 0
	return (ctrlReq(addr, ep, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, index, USB_DESCRIPTOR_STRING, langid, nbytes, nbytes, dataptr, 0));
	#endif

	return 0;
}

/**
 * \brief Set USB device address.
 *
 * \param oldaddr Current USB device address.
 * \param ep USB device endpoint number.
 * \param addr New USB device address to be set.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::setAddr(uint32_t oldaddr, uint32_t ep, uint32_t newaddr)
{
	#if 0
	TRACE_USBHOST(printf("   => USBHost::setAddr\r\n");)
	return ctrlReq(oldaddr, ep, bmREQ_SET, USB_REQUEST_SET_ADDRESS, newaddr, 0x00, 0x0000, 0x0000, 0x0000, 0, 0);
	#endif
	return 0;
}

/**
 * \brief Set configuration.
 *
 * \param addr USB device address.
 * \param ep USB device endpoint number.
 * \param conf_value New configuration value to be set.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t USBHost::setConf(uint32_t addr, uint32_t ep, uint32_t conf_value)
{
    	#if 0
		return (ctrlReq(addr, ep, bmREQ_SET, USB_REQUEST_SET_CONFIGURATION, conf_value, 0x00, 0x0000, 0x0000, 0x0000, 0, 0));
	#endif
	return 0;
}

#include <KeyboardController.h>

/**
 * \brief USB main task, responsible for enumeration and clean up stage.
 *
 * \note Must be periodically called from loop().
 */
void USBHost::Task(void)
{
	int loop = 1, ret = 0, max = 0, i = 0, j = 0;
	fd_set fdset, fdset_except;
	char dummy;
	extern int errno;
	struct usb_dev * pusb_dev = NULL;
	struct input_event ev[64];

	/* zero */
	FD_ZERO(&fdset);
	FD_ZERO(&fdset_except);


	/* Add elements - iterate through linked list of active elements */
	pusb_dev = phead;
	while(pusb_dev != NULL){
		max = max < pusb_dev->ihandle ? pusb_dev->ihandle : max;
		FD_SET(pusb_dev->ihandle, &fdset);

		pusb_dev = pusb_dev->pnext;
	}

	/* Select on the FD set - infinite timeout */
	ret = select(1 + max, &fdset, 0, NULL, NULL);

	/* Receive bytes */
	switch(ret){
		case -1:
			trace_error("critical fault during select errno=%d", errno);
			sleep(10);
			exit(-1);
			break;
		case 0:
			/* timeout */
			break;
		default:
			/* Process data */
			#if 0
				trace_debug("%s state change detected!", __func__);
			#endif

			pusb_dev = phead;

			/* Interrupt */
			while(pusb_dev != NULL){

				if (FD_ISSET(pusb_dev->ihandle, &fdset)){
					/* Read keyboard data */
					ret = read(pusb_dev->ihandle, ev,sizeof(ev));
					if ( ret < 0){
						fprintf(stderr, "unable to read from input device!\n");
						fflush(stdout);
						exit(-1);
					}

					for (i = 0; i < (int) (ret / sizeof(struct input_event)); i++) {
						if (EV_KEY == ev[i].type){
						    	#if 0
						        trace_debug("%ld.%06ld ",
						               ev[i].time.tv_sec,
						               ev[i].time.tv_usec);
							#endif
						        trace_debug("type %d code %d value %d",
						               ev[i].type,
						               ev[i].code, ev[i].value);

							// taking short-cut to get keyboard working this is ugly and I know it
							if(ev[i].value){
								_pCtrl->OnKeyDown(ev[i].value, ev[i].code);
							}else{
								_pCtrl->OnKeyUp(ev[i].value, ev[i].code);
							}
						}
					}
				}
				pusb_dev = pusb_dev->pnext;
			}
			break;
	}
}
