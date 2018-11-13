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

import variablepredictor as vp

import logging

class TimeChanged( Exception ):
	'''
	Класс, бросаемый в качестве исключения при изменении времени t_Fly
	'''
	def __init__( self, dt, time ):
		self.__dt = dt
		self.__time = time
	@property
	def dt( self ):
		return self.__dt
	@property
	def time( self ):
		return self.__time

class TimeWatcher( vp.LinearPredictor ):
	'''
	Класс для отслеживания изменений времени полета
	'''

	def __init__( self, sigma=0.1 ):
		self.__sigma = sigma
		self.__time = None
		self.__val = None
		self.__reliable = False
		super().__init__()

	def __reset( self ):
		self.__time = None
		self.__val = None
		self.__reliable = False

	def register_value( self, time: float, val: list ) -> bool:
		super().register_value( time, val )
		# при первом запуске только регистрируем значения
		if self.__time == None or self.__val == None:
			self.__time = time
			self.__val = val[0]
		else:
			self.__reliable = True
		# при следующих запусках вычисляем и проверяем разницу
		d_time = time - self.__time
		d_val = val[0] - self.__val
		if abs( d_time - d_val ) > 5 * self.__sigma:
			logging.warning( 'TimeWatcher: unexpected time step: expected {}, got {}'.\
				format( d_time, d_val ) )
			self.__reset()
			raise TimeChanged( abs( d_time - d_val ), time )


	def validate_time( self, real_time: float, vector_time: float ) -> bool:
		'''
		Проверяет, был ли скачок времени
		real_time - текущее системное время
		vector_time - условное время последнего полученного вектора
		Возвращает True если ход времени достоверен
		'''
		self.register_value( real_time, [ vector_time ] )


	@property
	def reliable( self ) -> bool:
		'''
		Возвращает признак достоверности данных
		'''
		return self.__reliable


