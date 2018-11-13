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
Очередь сообщений, отправляемых в имитатор навигационного сигнала
Скрипт содержит класс - очередь и классы сообщений
'''


import queue


class SimQueue:

	# время ожидания при операциях с очередью в секундах
	TIMEOUT = 1e-4

	def __init__( self ):
		self.__q = queue.Queue()

	def insert_command( self, command ):
		'''
		Добавляем элемент в очередь
		'''
		self.__q.put( command, timeout=self.TIMEOUT )

	def get_command( self ):
		'''
		Извлекаем один элемент из очереди и возвращаем его.
		Если очередь пуста, возвращаем None
		'''
		try:
			return self.__q.get_nowait()
		except queue.Empty:
			return None


class StopCmd:
	'''
	Выполняет перезапуск имитатора
	'''
	def call( self, obj ):
		obj.stop()

class StartAtNearestTimeCmd:
	'''
	Выполняет запуск имитатора в ближайщее подходящее время
	'''

	def __init__( self, time, ftime: float, predictors: dict ):
		'''
		time - время перезапуска
		ftime - количество секунд от запуска программы
		predictors - словарь, содержащий функции для предсказания параметров
			движения моделируемых КА в формате:
				{ идентификатор_КА: фукнция_предсказания }
		'''
		self.time = time
		self.ftime = ftime
		self.predictors = predictors
		print( 'StartAtNearestTimeCmd: Starting command executed...' )

	def call( self, obj ):
		obj.start_at_nearest_time( self.time, self.ftime, self.predictors )


class PredictAndSendVectorsCmd:
	'''
	Выполняет прогнозирование на опережающий момент и отправку вектора в
	имитатор навигационного сигнала
	'''

	def __init__( self, local_time, start_time, predict_dt: float, predictors ):
		'''
		local_time - текущее декретное время
		start_time - начало шкалы времени в модели движения
		predict_dt - опережение времени для определения прогноза вектора
		predictors - словарь, содержащий функции для предсказания параметров
			движения моделируемых КА в формате:
				{ идентификатор_КА: фукнция_предсказания }
		'''
		self.local_time = local_time
		self.start_time = start_time
		self.predict_dt = predict_dt
		self.predictors = predictors

	def call( self, obj ):
		obj.predict_and_send_vectors( \
			self.local_time, self.start_time, self.predict_dt, self.predictors )


