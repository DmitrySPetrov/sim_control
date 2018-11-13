######################################################################
#
#   This file is part of SimControl.
#
#   SimControl is free software: you can redistribute it and/or modify it under
#   the terms of the GNU General Public License as published by the Free
#   Software Foundation, either version 3 of the License, or (at your option)
#   any later version.
#
#   SimControl is distributed in the hope that it will be useful, but WITHOUT
#   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
#   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
#   more details.
#
#   You should have received a copy of the GNU General Public License along
#   with SimControl.  If not, see <https://www.gnu.org/licenses/>.
#
######################################################################

'''
Сервис, принимающий значение поправки ко времени
'''

import struct
import logging
import binascii
import socket, socketserver
import threading
import encodings.idna

class TimeDeltaSrv:

	DT_MSG_TYPE = 50
	DT_REQUEST_TYPE = 51
	SIGN_OFF_TYPE = 53
	DT_RECEIVED_TYPE = 60

	class RequestHandler( socketserver.BaseRequestHandler ):
		'''
		Обработчик входящего UDP-запроса
		'''
		def __init__( self, rcv_class, *args, **kwargs ):
			self.__rcv_class = rcv_class
			super( TimeDeltaSrv.RequestHandler, self ).__init__( *args, **kwargs )

		def handle( self ):
			data = self.request[0]
			# socket = self.request[1]
			self.__rcv_class.parse_msg( data )

	def __init__( self, srv_host, srv_port, clt_host, clt_port, req_timer_time=10.0 ):
		self.__srv_addr = ( srv_host, srv_port )
		self.__clt_addr = ( clt_host, clt_port )
		self.__clt_id = 'ASNsim'
		self.__num_by_type = {}
		self.__rcv_header = ( 0, 0, 0 )
		self.__rcv_status = 0
		self.__dt_setter = None
		self.__rcv_class = None
		self.__connected = False
		self.__dt_request_period = req_timer_time
		self.__req_timer = None
		self.__server = None

	def start_dt_request_timer( self ):
		if self.__connected:
			return
		self.request_for_dt()
		self.__req_timer = threading.Timer( self.__dt_request_period, self.start_dt_request_timer )
		self.__req_timer.start()

	def run( self ):
		'''
		Запустить слушающий сервер для обработки входящих пакетов
		'''
		self.__server = threading.Thread( target=self.run_server, daemon=True )
		self.__server.start()
		logging.info( 'TimeDeltaSrv: listening to {}'.format( self.__clt_addr ) )
		self.start_dt_request_timer()

	def stop( self ):
		self.__req_timer.cancel()
		self.__udp_srv.shutdown()

	def run_server( self ):
		self.__udp_srv = socketserver.UDPServer\
				( self.__clt_addr, self.__handle_request )
		self.__udp_srv.serve_forever()

	def __handle_request( self, *args, **kvargs ):
		return TimeDeltaSrv.RequestHandler( self, *args, **kvargs )

	def bind_dt_setter( self, dt_setter ):
		self.__dt_setter = dt_setter

	def __compose_id( self ):
		return '{};{}'.format( self.__clt_id, self.__clt_addr[1] )

	def __send_msg( self, type_, msg=b'' ):
		if type_ not in self.__num_by_type:
			self.__num_by_type[ type_ ] = 0
		self.__num_by_type[ type_ ] \
			= ( self.__num_by_type[ type_ ] + 1 ) % 256
		len_ = len( msg ) + 4
		head = struct.pack( '=BBH', type_, self.__num_by_type[ type_ ], len_ )
		logging.debug( 'TimeDeltaSrv: sending msg: {}'.\
				format( head+msg ) )
		# посылаем head+msg
		sock = socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
		sock.sendto( head+msg, self.__srv_addr )

	def request_for_dt( self ):
		s = self.__compose_id()
		msg = struct.pack( '={}s'.format( len( s ) ), s.encode( 'ascii' ) )
		self.__send_msg( self.DT_REQUEST_TYPE, msg )
		logging.info( 'TimeDeltaSrv: SENT request for DT' )

	def notify_dt_received( self ):
		msg = struct.pack( '=HBBH', self.__rcv_status, *self.__rcv_header )
		self.__send_msg( self.DT_RECEIVED_TYPE, msg )
		logging.info( 'TimeDeltaSrv: SENT receive notification' )

	def sign_off( self ):
		self.__send_msg( self.SIGN_OFF_TYPE )
		logging.info( 'TimeDeltaSrv: SENT sign off command' )

	def parse_msg( self, msg ):
		self.__connected = True
		logging.debug( 'TimeDeltaSrv: msg received: {}'.\
				format( binascii.hexlify( msg ) ) )
		type_, num, len_ = struct.unpack( '=BBH', msg[:4] )
		if len( msg ) != len_:
			logging.warning( 'TimeDeltaSrv: message ignored ' \
					+ 'due to inappropriate length.' \
					+'In message: {}, exist: {}'.format( len_, len( msg ) ) )
			self.__rcv_status = -3
			return
		if type_ not in self.__num_by_type:
			self.__num_by_type[ type_ ] = num
		elif num == self.__num_by_type[ type_ ]:
			logging.warning( 'TimeDeltaSrv: message duplication. ' \
					+'CODE: {}'.format( num ) )
			self.__rcv_status = -2
			return
		if type_ == self.DT_MSG_TYPE:
			self.parse_dt_msg( msg[4:] )
		else:
			self.__rcv_status = -1
			logging.warning( 'TimeDeltaSrv: invalid message type {}'. \
					format( type_ ) )

	def parse_dt_msg( self, msg ):
		logging.debug( 'TimeDeltaSrv: RECV dt message' )
		if len( msg ) != 10:
			logging.warning( 'TimeDeltaSrv: dt_msg: invalid len: {}'. \
					format( len( msg ) + 4 ) )
			self.__rcv_status = -3
			return
		rel, dt = struct.unpack( '=hd', msg )
		if rel not in ( 0, 1 ):
			logging.warning( 'TimeDeltaSrv: dt_msg reliability flag value: {}'. \
					format( rel ) )
			self.__rcv_status = -4
			return
		if self.__dt_setter:
			logging.info( 'TimeDeltaSrv: trying to set DT value: {}({})'.\
				format( dt, rel ) )
			self.__dt_setter( dt, rel )
		self.__rcv_status = 0

		self.notify_dt_received()

