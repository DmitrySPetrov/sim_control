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

import numpy as np
import mathutils
import math

# Храним в таблице значения факториалов
fac = [ 1 ]
for i in range( 10 ):
	fac += [ fac[-1] * ( i+1 ) ]


class PredictorException( Exception ):
	def __init__( self, *v, **kv ):
		super().__init__( self, *v, **kv )

class BigExtrapolateTime( PredictorException ):
	def __init__( self, dt ):
		super().__init__( self, 'Too big time delta to extrapolate: {}'.\
				format( dt ) )


class LinearPredictor:
	'''
	Класс для аппроксимации линейных величин с известной производной
	'''

	def __init__( self, der: float=1.0 ):
		self.__der = der
		self.__val = None
		self.__time = 0.0

	def register_value( self, time: float, val: list ) -> None:
		self.__time = time
		self.__val = val[0]

	@property
	def value( self ) -> float:
		'''
		Возвращает последнее значение величины
		'''
		if self.__val == None:
			return 0.0
		return self.__val

	def diff( self, num: int ) -> float:
		'''
		Возвращает производную величины порядка num
		'''
		if self.__val == None:
			return 0.0
		if num == 0:
			return self.__val
		elif num == 1:
			return self.__der
		else:
			return 0.0

	def extrapolate( self, time: float, dim_o: int=None ) -> list:
		'''
		Возвращает массив величины и производных на заданное время
		'''
		if self.__val == None:
			return [ 0.0, 0.0 ]
		return [ self.__val + self.__der * ( time - self.__time ), self.__der ]

	@property
	def reliable( self ) -> bool:
		'''
		Возвращает признак достоверности данных
		'''
		return self.__val != None

	def restart( self ):
		self.__reliable = False
		self.__val = None


class VariablePredictor:
	'''
	Класс, позволяющий предсказать значение величины
	Хранит величины производных, умеет экстраполировать величину к
	заданному времени
	'''

	# дисперсия шума наблюдений
	OBSERVATION_NOISE_DISP = 1e-10

	def init_matrices( self ):
		pass

	def init_filter( self ):
		# Коэффициенты полинома
		self.__x = None
		# Ковариационная матрица
		self.__P = self.__P_mul * np.identity( self.__dim )
		# Флаг достоверности вектора состояния
		self.__reliable = False

	def __init__( self, higher_diff: int, disp: float, P_mul: float=1.0 ):
		'''
		higher_diff - номер старшей хранимой производной
		disp - дисперсия управляющего воздействия
		P_mul - множитель к начальному значению ковариационной матрицы
		'''
		assert( higher_diff > 1 )
		self.__dim = higher_diff + 1
		self.__time = 0.0
		self.__P_mul = P_mul

		self.init_filter()
		# Дисперсия управляющего воздействия
		self.__disp = disp
		# Вектор со среднеквадратичными отклонениями, определяющий достоверность
		self.__sigma2_rel = []
		return

	def __calcF( self, t ):
		'''
		Вычисляем матрицу эволюции
		dt - интервал времени эволюции
		'''
		F = np.identity( self.__dim )
		t_pow = 1.0		# t в нулевой степени
		for i in range( self.__dim ):
			for j in range( self.__dim - i ):
				F[ j ][ i+j ] = t_pow
			t_pow *= t / ( i+1 )
		return F

	def __calcG( self, t, dim_i ):
		'''
		Вычисляем матрицу управления
		dt - интервал времени, для которого вычисляются матрицы
		dim_i - размерность вектора входной информации
		'''
		G = np.zeros( ( self.__dim, 1 ) )
		t_pow = 1.0		# t в нулевой степени
		for i in reversed( range( dim_i ) ):
			G[ i ] = t_pow
			t_pow *= t / ( i+1 )
		return G

	def __calcFG( self, t, dim_i ):
		'''
		Вычисляем матрицу эволюции и матрицу управления
		dt - интервал времени, для которого вычисляются матрицы
		dim_i - размерность вектора входной информации
		'''
		F = np.identity( self.__dim )
		G = np.zeros( ( self.__dim, 1 ) )
		t_pow = 1.0		# t в нулевой степени
		for i in range( self.__dim ):
			for j in range( self.__dim - i ):
				F[ j ][ i+j ] = t_pow
			if i <= dim_i:
				G[ dim_i - i ] = t_pow
			t_pow *= t / ( i+1 )
		return F, G

	def register_value( self, time: float, val: list,
			F=None, F_t=None, GGt=None, H=None, H_t=None ) -> None:
		'''
		Зарегистрировать новое значение величины на указанный момент
		времени
		'''
		dt = time - self.__time
		self.__time = time

		# обрезаем наблюдаемый вектор до нужного размера если необходимо
		if len( val ) > self.__dim:
			vec_ = val[ :self.__dim ]
		else:
			vec_ = val
		# наблюдаемый вектор
		vec = np.array( vec_ ).reshape( ( len( vec_ ), 1 ) )

		if self.__x is None:
			self.__x = vec.copy()
			self.__x.resize( ( self.__dim, 1 ) )
			self.__reliable = False
			return

		# матрицы для осуществления измерений
		if F is None and GGt is None:
			F,G = self.__calcFG( dt, len( vec_ ) )
			GGt = G.dot( G.transpose() )
		elif F is None:
			F = self.__calcF( dt, len( vec_ ) )
		elif GGt is None:
			G = self.__calcG( dt, len( vec_ ) )
			GGt = G.dot( G.transpose() )
		if F_t is None:
			F_t = F.transpose()
		if H is None:
			H = np.eye( len( vec_ ), self.__dim )
		if H_t is None:
			H_t = H.transpose()
		# матрица шума наблюдения
		R = self.OBSERVATION_NOISE_DISP * np.identity( len( vec_ ) )

		# начальный прогноз
		xpred = F.dot( self.__x )
		Ppred = F.dot( self.__P ).dot( F_t ) + self.__disp * GGt

		# оценка невязки
		y = vec - H.dot( xpred )
		S = H.dot( Ppred ).dot( H_t ) + R

		# коррекция
		K = Ppred.dot( H_t ).dot( np.linalg.inv( S ) )
		self.__x = xpred + K.dot( y )
		self.__P = ( np.identity( self.__dim ) - K.dot( H ) ).dot( Ppred )
		# self.__x теперь содержит новые отфильтрованные значения

		# оцениваем расхождение: вычисляем матрицу ковариации
		Cov = np.cov( np.hstack( ( xpred, self.__x ) ) )
		# Проверяем, получили ли достоверность
		if len( self.__sigma2_rel ) == 0:
			self.__reliable = False

		else:
			for i in range( len( self.__sigma2_rel ) ):
				if Cov[i][i] > self.__sigma2_rel[i]:
#					print( self.__x )
#					print( xpred )
#					print( Cov[i][i], self.__sigma2_rel[i] )
					self.__reliable = False
					break
			else:
				self.__reliable = True

	@property
	def time( self ) -> float:
		'''
		Возвращает время, на которое была зафиксирована последняя величина
		'''
		if self.__time == None:
			return 0.0
		return self.__time

	@property
	def value( self ) -> float:
		'''
		Возвращает последнее значение величины
		'''
		if self.__x is None:
			return 0.0
		return self.__x[0][0]

	def diff( self, num: int ) -> float:
		'''
		Возвращает производную величины порядка num
		'''
		if self.__x is None:
			return 0.0
		return self.__x[ num ][0]

	def extrapolate( self, time: float, dim_o: int=None ) -> list:
		'''
		Возвращает массив величины и производных на заданное время
		'''
		if self.__x is None:
			return [ 0.0 ] * dim_o
		dt = time - self.__time
		F = self.__calcF( dt )
		res = F.dot( self.__x )
		if dim_o == None:
			dim_o = self.__dim
		return res.transpose()[0].tolist()[:dim_o]

	def average( self, time: float, time_start: float=None ) -> float:
		'''
		Вычисляет среднее значение величины между time_start и time
		Если time_start == None, вместо него используется время последнего
		зарегистрированного значения
		'''
		if self.__x is None:
			return 0.0
		global fac
		if time_start == None:
			time_start = self.__time
		dt = time - time_start
		res = self.__x[0][0]
		_dt = dt
		fac = 1.0
		for i in range( 1, self.__dim ):
			res += dt * self.__x[i][0] / fac
			dt *= dt
			fac *= i + 1
		return res

	def set_reliable_condition( self, sigma: list ) -> None:
		self.__sigma2_rel = list( s*s for s in sigma )

	@property
	def reliable( self ) -> bool:
		'''
		Возвращает признак достоверности данных
		'''
		return self.__reliable

	def restart( self ):
		self.init_filter()


class QuaternionPredictor:
	'''
	Класс, позволяющий предсказать кватернион разворота в предположении
	постоянства угловой скорости
	'''
	# TODO: реализовать работу класса
	def __init__( self ):
		self.__wX = VariablePredictor( 3, 1e-12, 100.0 )
		self.__wY = VariablePredictor( 3, 1e-12, 100.0 )
		self.__wZ = VariablePredictor( 3, 1e-12, 100.0 )
		self.__q = None

	def register_value( self, time: float, val: list ):
		'''
		val - кватернион разворота и угловые скорости
		'''
		_l_val = len( val )
		if _l_val >= 7:
			_q = val[ 0:3 ]
			_wX = val[ 4:4 ]
			_wY = val[ 5:5 ]
			_wZ = val[ 6:6 ]
		else:
			_wX = [ 0.0 ]
			_wY = [ 0.0 ]
			_wZ = [ 0.0 ]
			if _l_val >= 4:
				_q = val[ 0:4 ]
			elif _l_val < 4:
				_q = val + [ 0.0 ] * ( 4-_l_val )
			else:
				_q = val
		# Кватернион просто переписываем
		self.__q = mathutils.Quaternion( _q )
		# Угловые скорости аппроксимируем
		self.__wX.register_value( time, _wX )
		self.__wY.register_value( time, _wY )
		self.__wZ.register_value( time, _wZ )

	@property
	def value( self ) -> list:
		return self.__q

	def diff( self, num: int ) -> list:
		if num != 0:
			return [ 0.0 ] * 4
		else:
			return self.__q

	def extrapolate( self, time: float, dim_o: int=None ) -> list:
		# вычисляем угол поворота исходя из средних значений угловых скоростей
		_aX = self.__wX.average( time ) * ( time - self.__wX.time )
		_aY = self.__wY.average( time ) * ( time - self.__wY.time )
		_aZ = self.__wZ.average( time ) * ( time - self.__wZ.time )
		_a = math.sqrt( _aX*_aX + _aY*_aY + _aZ*_aZ )
		q = mathutils.Euler( ( _aX, _aY, _aZ ), 'YXZ' ).to_quaternion() * self.__q
		return [ q.w, q.x, q.y, q.z ]

	@property
	def wX( self ):
		return self.__wX

	@property
	def wY( self ):
		return self.__wY

	@property
	def wZ( self ):
		return self.__wZ

	@property
	def reliable( self ) -> bool:
		'''
		Возвращает признак достоверности данных
		'''
		return \
			self.__wX.reliable \
			and self.__wY.reliable \
			and self.__wZ.reliable

	def restart( self ):
		self.__reliable = False
		self.__q = None


class OrbitalMovementPredictor:
	'''
	Класс, позволяющий предсказать орбитальное движение
	Хранит приближенные значения гравитационного коэффициента и коэффициента
	аэродинамического трения
	'''

	# дисперсия шума наблюдений
	OBSERVATION_NOISE_DISP = 1e-10

	# максимальное время экстраполяции
	MAX_EXTRAPOLATE_TIME = 100.0

	# Гравитационная постоянная
	PHYS_G = 6.67408e-11
	# Масса Земли по-умолчанию
	EARTH_M = 5.97219e+24
	# Их произведение для Земли
	EARTH_G = PHYS_G * EARTH_M
	# Аэродинамическое трение
	AERO_BETA = 1.0e-10
	# Номинальная орбитальная скорость
	MOD_V = 7.6e+3
	# Номинальное аэродинамическое ускорение
	AERO_A = AERO_BETA * MOD_V

	def init_matrices( self ):
		self.__Q = np.zeros( ( 8,8 ) )
		self.__Q[0][0] = 0e-6
		self.__Q[1][1] = 0e-6
		self.__Q[2][2] = 0e-6
		self.__Q[3][3] = 1e-9
		self.__Q[4][4] = 1e-9
		self.__Q[5][5] = 1e-9
		self.__Q[6][6] = 1e+10
		self.__Q[7][7] = 1e-15

		self.__P = 1e10 * np.identity( 8 )
		self.__P[0][0] = 1e10
		self.__P[1][1] = 1e10
		self.__P[2][2] = 1e10
		self.__P[3][3] = 1e10
		self.__P[4][4] = 1e10
		self.__P[5][5] = 1e10
		self.__P[6][6] = 1e20
		self.__P[7][7] = 1e-10

		self.__H = np.array( [\
				[1,0,0,0,0,0,0,0],
				[0,1,0,0,0,0,0,0],
				[0,0,1,0,0,0,0,0],
				[0,0,0,1,0,0,0,0],
				[0,0,0,0,1,0,0,0],
				[0,0,0,0,0,1,0,0]\
			] )
		self.__R = 1.0e-10 * np.identity( 6 )

	def init_filter( self ):
		self.init_matrices()
		self.__x = None
		# Флаг достоверности вектора состояния
		self.__reliable = False
 
	def __init__( self, higher_diff: int, disp: float, P_mul: float=1.0 ):
		'''
		higher_diff - номер старшей хранимой производной
		disp - дисперсия управляющего воздействия
		P_mul - множитель к начальному значению ковариационной матрицы
		'''
		self.predX = VariablePredictor( higher_diff, disp, P_mul )
		self.predY = VariablePredictor( higher_diff, disp, P_mul )
		self.predZ = VariablePredictor( higher_diff, disp, P_mul )

		self.init_filter()

		# Вектор со среднеквадратичными отклонениями, определяющий достоверность
		self.__sigma2_rel = []

		# Максимальная величина шага по времени в секундах
		self.__max_deltaT = 1.0


	@staticmethod
	def __calcF( phi, tau ):
		'''
		Вычисляем матрицу эволюции
		phi - вектор состояния
		tau - интервал времени эволюции
		'''
		r2 = sum( a[0]*a[0] for a in phi[:3] )
		r = math.sqrt( r2 )
		r3 = r*r2
		v = math.sqrt( sum( a[0]*a[0] for a in phi[3:6] ) )
		_a = tau * phi[6][0] / r3
		_b = tau * phi[7][0]
		_c = _a * 3 / r2

		F = np.identity( 8 )
		for i in range(3):
			F[i][i+3] = tau

			for j in range(3):
				F[i+3][j] = _c * phi[i][0]*phi[j][0]

			F[i+3][i] += -_a
			F[i+3][i+3] += 1 - _b*v

			F[i+3][6] = -tau * phi[i][0] / r3
			F[i+3][7] = -tau * phi[i+3][0]

		return F

	@staticmethod
	def __calcW( phi, tau ):
		r2 = sum( a[0]*a[0] for a in phi[:3] )
		r = math.sqrt( r2 )
		r3 = r*r2
		v = math.sqrt( sum( a[0]*a[0] for a in phi[3:6] ) )
		_a = phi[6][0] / r3
		_b = phi[7][0] * v
		W = np.zeros( ( 8,1 ) )
		for i in range(3):
			W[i] = phi[i+3][0]
			W[i+3] = -_a * phi[i] - _b * phi[i+3]
		return W

	@staticmethod
	def __calc_deriv( phi, dphi=True ):
		'''
		Вычисляем производную ВС
		dPhi == True: вычисляем производную для всего вектора
		dPhi == False: вычисляем скорость, ускорение, джерк
		'''
		x = phi[:3][:]
		v = phi[3:6][:]
		mu = phi[6][0]
		beta = phi[7][0]
		r2 = sum( _x*_x for _x in x )
		r3 = r2 ** 1.5
		accel = ( - mu * x / r3 - beta * v )
		if dphi:
			res_x = v
			res_v = accel
			return np.vstack( ( res_x, res_v, np.array( [[0.0],[0.0]] ) ) )
		else:
			res_x = x
			res_v = v
			res_a = accel
			res_j = - v*mu/r3 \
					+ 3*mu*x*np.dot(np.transpose(x),v)/(r2*r3) \
					- beta*accel
#			print( sum( _v[0]*_v[0] for _v in res_v )**0.5 )
#			print( sum( _a[0]*_a[0] for _a in res_a )**0.5 )
#			print( sum( _j[0]*_j[0] for _j in res_j )**0.5 )
			return np.vstack( ( res_x, res_v, res_a, res_j ) )

	def register_value( self, time: float, val: list ) -> None:
		'''
		time - условное время, на которое вычислен вектор
		val - список вида [X,Y,Z,VX,VY,VZ]
		'''
		assert( len( val ) == 6 )
		# Делаем из него массив np.array
		vec = np.array( val ).reshape( ( 6, 1 ) )

		if self.__x is None:
			self.__x = vec.copy()
			self.__x.resize( ( 8, 1 ) )
			self.__x[6] = self.EARTH_G
			self.__x[7] = self.AERO_BETA
			self.__time = time
			self.__reliable = False
			return

		dt = time - self.__time
		self.__time = time

		tau = dt

		# Вычисляем матрицы
		F = self.__calcF( self.__x, tau )
		W = self.__calcW( self.__x, tau )
		# Вычисляем прогноз
		x_ = self.__extrapolate( self.__x, tau )
#		print( F )
#		print( self.__P )
#		print( W )
#		print( self.__Q )
		P_ = F.dot( self.__P.dot( np.transpose( F ) ) ) \
			+ self.__Q
#			+ W.dot( self.__Q.dot( np.transpose( W ) ) )
		# Отклонение
		z = vec - self.__H.dot( x_ )
		S = self.__H.dot( P_ ).dot( np.transpose( self.__H ) ) + self.__R
		# Матрица усиления
		K = P_.dot( np.transpose( self.__H ) ).dot( np.linalg.inv( S ) )
		# Коррекция прогноза
		self.__x = x_ + K.dot( z )
		self.__P = ( np.identity(8) - K.dot( self.__H ) ).dot( P_ )

		# Оцениваем, есть ли достоверность
		Cov = np.cov( np.hstack( ( x_, self.__x ) ) )

		if len( self.__sigma2_rel ) == 0:
			self.__reliable = False

		else:
			for i in range( len( self.__sigma2_rel ) ):
				if Cov[i][i] > self.__sigma2_rel[i]:
#					print( self.__x )
#					print( xpred )
#					print( Cov[i][i], self.__sigma2_rel[i] )
					self.__reliable = False
					break
			else:
				self.__reliable = True


	def set_reliable_condition( self, sigma: list ) -> None:
		self.__sigma2_rel = list( s*s for s in sigma )

	@property
	def reliable( self ) -> bool:
		'''
		Возвращает признак достоверности данных
		'''
		return self.__reliable

	def reset_x( self ) -> None:
		'''
		Сбрасывает начальный ВС
		'''
		self.__x = None

	def __extrapolate_euler( self, vec, dt ):
		'''
		Интегрируем методом Эйлера
		'''
		deriv = self.__calc_deriv( vec )
		res = vec.copy()
		res += deriv * dt
		return res

	def __extrapolate_rk4( self, vec, h ):
		'''
		Интегрируем методом Рунге-Кутты 4го порядка
		'''
		k1 = self.__calc_deriv( vec )
		k2 = self.__calc_deriv( vec + h/2*k1 )
		k3 = self.__calc_deriv( vec + h/2*k2 )
		k4 = self.__calc_deriv( vec + h*k3 )
		return vec + h/6 * ( k1 + 2*k2 + 2*k3 + k4 )

	def __extrapolate( self, vec, DeltaT, method='rk4' ):
		if method == 'rk4':
			extrapolate_fun = self.__extrapolate_rk4
		elif method == 'euler':
			extrapolate_fun = self.__extrapolate_euler
		if abs( DeltaT ) > self.MAX_EXTRAPOLATE_TIME:
			# Кидаем исключение если слишком далеко экстраполировать
			raise BigExtrapolateTime( DeltaT )
		res = vec.copy()
		dt_ = (-1.0,1.0)[ DeltaT > 0 ] * self.__max_deltaT
		while abs( DeltaT ) > self.__max_deltaT:
			res = extrapolate_fun( res, dt_ )
			DeltaT -= dt_
		return extrapolate_fun( res, DeltaT )

	def extrapolate( self, time: float, dim_o: int=None ) -> list:
		'''
		Возвращает массив величины и производных на заданное время
		'''
		if self.__x is None:
			return [ 0.0 ] * 12
		# Прогнозируем движение на заданное время
		DeltaT = time - self.__time
		res = self.__extrapolate( self.__x, DeltaT )
		#print( self.__x )
		#print( res )
		# Вычисляем производные
		return self.__calc_deriv( res, dphi=False )

	def restart( self ):
		self.init_filter()


if __name__ == '__main__':
	print( fac )
	print()
	_v = VariablePredictor( 5, 1e-12 )
	_v.set_reliable_condition( [ 1e-1, 1e-2 ] )
	t = 1.0
	dt = 1.0
	x,v,a,j,j2 = 6800.0e3, 0.0, -9.81, 0.00, 1.4152e-5

	R = 6800.0e3
	W = ( 9.81 / R ) ** 0.5

	rel_prev = False

	for i in range( 1500 ):
		def vec2str( v ):
			return '{:10.1f} {:9.3f} {:6.3f} {:9.6f}'.format( *v[:4] )

		# Величина, на которую прогнозируем
		DT = 20.0

		# Регистрируем предыдущие значения
		_v.register_value( t, ( x, v ) )

		t += dt
		x = R * math.cos( W * t )
		v = - R * W * math.sin( W * t )
		a = - R * W * W * math.cos( W * t )
		j = R * W * W * W * math.sin( W * t )

		print( 'T={:8.2f}'.format( t ) )
		print( 'Real values are: x={:10.1f}; x\'={:9.3f}; x"={:6.3f}; x"\'={:9.6f}'.\
			format( x, v, a, j ) )
		print( 'Prediction to T: ' + vec2str( _v.extrapolate( t ) ) )
		print( 'Prediction to t={}: '.format( t+dt ) \
			+ vec2str( _v.extrapolate( t+dt ) ) )
		print( 'Prediction to t={}: '.format( t+DT ) \
			+ vec2str( _v.extrapolate( t+DT ) ) )

		if _v.reliable and not rel_prev:
			print()
			print( 'Reliable!' )
		if not _v.reliable and rel_prev:
			print()
			print( 'Not reliable!' )
		rel_prev = _v.reliable

		print()
		print()

