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
Скрипт эмулирует работу сетевого-сервиса имитатора АСН:
1) выдает статусы в нужной последовательности
2) принимает вектор состояния
3) умеет включаться и выключаться
'''

from lxml import etree
from datetime import datetime, timedelta
import re
import socketserver
from threading import Lock, Timer
import time

class SimSimulator:

	TIMESTAMP_RE = re.compile( '(?=(\d+) )?\d{2}:\d{2}:\d{2}(?=\.\d{1-3})?' )
	TIMESTAMP_NUM_RE = re.compile( '(\d+(?=\.\d*))' )

	START_TIME_FMT = '(\d{{2}})-({})-(\d{{2,4}})\s+(\d{{2}}):(\d{{2}}):(\d{{2}})'

	MONTHS = [ 'JAN', 'FEB', 'MAR', 'APR', 'MAY', 'JUN', 'JUL', 'AUG', 'SEP', 'OCT', 'NOV', 'DEC' ]

	def __init__( self, period=0.25 ):
		self.__status = 1
		# начальное условное время на момент запуска имитатора
		self.__tng_start_time = None
		# время включения питания имитатора
		self.__turnon_time = datetime.now()
		# время от выдачи команды "AR"
		self.__ar_time = None
		# время от выдачи команды "запуск"
		self.__run_time = None
		# период обновления статуса
		self.__period = period
		# константы для парсинга строк
		months_str = '|'.join( self.MONTHS )
		self.START_TIME_RE = re.compile( self.START_TIME_FMT.format( months_str ) )
		self.MONTH_DICT = { s: n+1 for n,s in enumerate( self.MONTHS ) }
		# блокировка для обновления статуса
		self.__status_lock = Lock()
		# признак работы периодической части класса
		self.__periodic_work = None

	@property
	def status( self ):
		with self.__status_lock:
			return self.__status
	@status.setter
	def status( self, value ):
		with self.__status_lock:
			self.__status = value

	def __compose_status_msg( self ):
		resp = etree.Element( 'msg' )
		stat = etree.Element( 'status' )
		stat.text = ' {:d} '.format( self.status )
		resp.append( stat )
		res = etree.tounicode( resp, pretty_print=True )
		return res

	def handle_request( self, data: str ):
		'''
		Обрабатывает ASCII-запрос
		'''
		items = data.split( ',' )
		# первый элемент - время или прочерк
		while True:
			if items[0] == '-':
				_time = timedelta( seconds=0 )
				items = items[ 1: ]
				break

			m = self.TIMESTAMP_NUM_RE.match( items[0] )
			if m:
				print( m )
				_time = timedelta( seconds=float( items[0] ) )
				items = items[ 1: ]
				break

			m = self.TIMESTAMP_RE.match( items[0] )
			if m:
				print( m )
				# TODO: реализовать
				_time = timedelta( seconds=float( items[0] ) )
				items = items[ 1: ]
				break

			# теперь первый элемент - команда
			_time = timedelta( seconds=0.0 )
			break

		cmd = items[0]
		print( cmd )

		items = items[ 1: ]

		# запрос статуса
		if cmd == 'NULL':
			return self.__compose_status_msg()

		# установка начального времени
		elif cmd == 'START_TIME':
			m = self.START_TIME_RE.match( items[0] )
			if m:
				self.__tng_start_time = datetime( \
					int( m.group(3) ), self.MONTH_DICT[ m.group(2) ], int( m.group(1) ), \
					int( m.group(4) ), int( m.group(5) ), int( m.group(6) ) )
			return self.__compose_status_msg()

		elif cmd == 'AR':
			if self.status != 2:
				return self.__compose_status_msg()
			self.status = 3
			self.__ar_time = datetime.now()
			time.sleep( 5.1 )
			self.status = 4
			return self.__compose_status_msg()

		elif cmd == 'RU':
			self.__run_time = datetime.now() + timedelta( seconds=0.900 )
			return self.__compose_status_msg()

		elif cmd == 'EN':
			print( items[0] )
			if len( items ) > 0 and int( items[0] ) == 1:
				self.status = 2
			else:
				self.status = 7
			return self.__compose_status_msg()
		else:
			if self.__run_time:
				print( 'Sim time: {:12.3f}'.\
					format( ( datetime.now() - self.__run_time ).total_seconds() ) )
			return self.__compose_status_msg()

	def start_periodic( self ):
		self.__periodic_work = True
		self.periodic()

	def periodic( self ):
		'''
		Периодически вызываемый метод класса
		Осуществляет обновление статуса
		'''
		now = datetime.now()
		# Обновляем статус 1 на 2
		if self.status == 1:
			self.__ar_time = None
			self.__run_time = None
			if ( now - self.__turnon_time ).total_seconds() > 5:
				self.status = 2

		# Подготовка имитатора
		elif self.status == 3:
			self.__run_time = None
			if self.__ar_time:
				if ( now - self.__ar_time ).total_seconds() > 10:
					self.status = 4

		# Запуск имитатора
		elif self.status == 4:
			if self.__run_time:
				print( 'Nearly to start!' )
				if ( now - self.__run_time ).total_seconds() > 0:
					self.status = 5

		# запускаем следующую итерацию
		if self.__periodic_work:
			Timer( self.__period, self.periodic ).start()

	def stop_periodic( self ):
		self.__periodic_work = False


class SimRequestHandler( socketserver.BaseRequestHandler ):
	def handle( self ):
		self.data = self.request.recv( 1024 ).decode( 'ascii' ).strip()
		print( '\nRequest:\n{}\n'.format( self.data ) )
		resp = self.server.simsim.handle_request( self.data )
		print( 'Response:\n{}\n'.format( resp ) )
		#resp = resp.replace( '  ', '\t' ) + '\r'
		#print( resp.encode( 'ascii' ) )
		self.request.send( resp.encode( 'ascii' ) )

class SimRequestServer( socketserver.TCPServer ):
	def __init__( self, simsim, *args, **kwargs ):
		self.simsim = simsim
		super().__init__( *args, **kwargs )


if __name__ == '__main__':

	simsim = SimSimulator()
	simsim.start_periodic()

	HOST = 'localhost'
	PORT = 15650

	try:
		server = SimRequestServer( simsim, ( HOST, PORT ), SimRequestHandler )
		server.serve_forever()
	except OSError as e:
		raise e
	finally:
		simsim.stop_periodic()
		server.shutdown()
		server.server_close()

