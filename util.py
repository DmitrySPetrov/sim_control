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

from datareceiver import BufferError
import simdispatcher

import struct
import queue
import datetime
import threading
import logging
import traceback

class ControlSumCalculator:
	'''
	Класс для вычисления контрольной суммы
	'''

	def __init__( self, byte_order_little=True ):
		logging.debug( 'Creating ControlSumCalculator instance' )
		if byte_order_little:
			self.__bo_sign = '<'
			self.__bo = 'little'
		else:
			self.__bo_sign = '>'
			self.__bo = 'big'

	def __call__( self, buf ):
		w_size = len( buf ) // 2
		r = struct.unpack_from( self.__bo_sign + 'H'*w_size, buf )
		sum = 0
		for i,s in enumerate( r ):
			if i != 1:
				sum += s
			while sum > 0xffff:
				sum = sum + 1 - 0x10000
		if len( buf ) % 2 != 0:
			sum += int.from_bytes( [ buf[-1], 0 ], byteorder=self.__bo, \
					signed=False )
			while sum > 0xffff:
				sum = sum + 1 - 0x10000
		if sum != r[1]:
			raise BufferError( 'Invalid control sum {}, expected {}'.\
				format( sum, r[1] ) )


class DataQueue:

	# время ожидания при операциях с очередью в секундах
	TIMEOUT = 1e-4

	def __init__( self ):
		logging.debug( 'Creating DataQueue instance' )
		self.__q = queue.Queue()

	def packer( self, params: dict ):
		'''
		Выполняет упаковку параметров из словаря в структуру данных.
		Должен быть переопределен
		'''
		return ()

	def insert_params( self, param: dict ):
		'''
		Вызывается при получении нового сообщения от модели движения.
		Выполняет преобразование формата хранения параметров и
		кладет их в очередь.
		'''
		self.__q.put( self.packer( param ), timeout=self.TIMEOUT )

	def get_params( self ):
		'''
		Извлекаем один элемент из очереди и возвращаем его.
		Если очередь пуста, возвращаем None
		'''
		try:
			# get_nowait потому что просто get ждет появления нового элемента
			return self.__q.get_nowait()
		except queue.Empty:
			return None


class VectorProcessorCaller:
	'''
	'''

	def __init__( self, vp ):
		logging.debug( 'Creating VectorProcessorCaller instance' )
		self.vp = vp
		self.start_time = None
		self._time_advance = 0.0

	@property
	def time_advance( self ) -> float:
		'''
		Опережение по времени, на которое выдается вектор состояния в имитатор
		навигационного сигнала
		'''
		return self._time_advance
	@time_advance.setter
	def time_advance( self, value: float ) -> None:
		self._time_advance = value

	def set_train_time( self, time: float ) -> None:
		'''
		Устанавливает тренировочное время на текущий момент
		'''
		self.start_time = \
			datetime.datetime.now() - datetime.timedelta( seconds=time )

	def __time_to_float( self, now ) -> float:
		return \
			( now - self.start_time ).total_seconds() + self._time_advance

	def __call__( self ) -> None:
		'''
		Осуществляет вызов функции VectorProcessor.worker( time ),
		где вместо time подставляется условное время, опережающее текущее на
		величину self.
		'''
		assert self.start_time != None
		time = datetime.datetime.now()
		self.vp.worker( self.__time_to_float( time ), time )


class SimControllerWorkerCaller:
	def __init__( self, sc, timeperiod=0.05 ):
		logging.debug( 'Creating SimControllerWorkerCaller instance' )
		self.sc = sc
		self.timeperiod = timeperiod
		self.running = False
		self.has_connection = True

	def run_once( self ):
		if self.running:
			try:
				do = self.sc.worker()
				if do and ( not self.has_connection ):
					self.has_connection = True
					logging.info( 'SimControllerWorkerCaller: Connection to simulator restored' )

			except ( ConnectionError, simdispatcher.CouldNotConnectToSimulator ):
				if self.has_connection:
					self.has_connection = False
					logging.warning( 'SimControllerWorkerCaller: Connection to simulator lost' )

			except Exception as e:
				p = traceback.format_exc()
				logging.error( 'Error in SimControllerWorkerCaller: {}\n{}'.format( e, p ) )

			threading.Timer( self.timeperiod, self.run_once ).start()

	def start( self ):
		self.running = True
		self.run_once()

	def stop( self ):
		self.running = False


class GetStatusCaller:

	def __init__( self, sd ):
		logging.debug( 'Creating GetStatusCaller instance' )
		self.sd = sd
		self.has_connection = True

	def __call__( self ) -> None:
		try:
			self.sd.get_status()
			if not self.has_connection:
				self.has_connection = True
				logging.info( 'GetStatusCaller: Connection to simulator restored' )

		except ( ConnectionError, simdispatcher.CouldNotConnectToSimulator ):
			if self.has_connection:
				self.has_connection = False
				logging.warning( 'GetStatusCaller: Connection to simulator lost' )

		except Exception as e:
			logging.error( 'Error in GetStatusCaller: {}'.format( e ) )


class CoordinateScaler:
	'''
	Выполняет масштабирование координат
	'''
	def __init__( self ):
		self.__scale = {}

	def set_scale( self, coord_names, scale ):
		for n in coord_names:
			self.__scale[ n ] = scale

	def __call__( self, params: dict ):
		for k,v in self.__scale.items():
			params[ k ] = tuple( a * v for a in params[ k ] )

