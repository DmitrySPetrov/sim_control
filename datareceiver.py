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

import struct
import socketserver
import logging

class DataIntersection( Exception ):
    pass

class InvalidBuffer( Exception ):
    pass

class BufferError( Exception ):
    pass

class RequestHandler( socketserver.BaseRequestHandler ):
    '''
    Обработчик входящего UDP-запроса
    '''
    def __init__( self, rcv_class, *args, **kwargs ):
        self.__rcv_class = rcv_class
        super( RequestHandler, self ).__init__( *args, **kwargs )

    def handle( self ):
        data = self.request[0]
        # socket = self.request[1]
        self.__rcv_class.parse_buffer( data )

class DataReceiver:
    '''
    Класс для приема данных по сети
    Создается слушающий UDP сокет, на который периодически отсылаются
        данные, содержащий информацию согласно протоколу
    '''

    def __init__( self, ip: str, port: int, byte_order_little: bool=True ):
        '''
        ip - адрес компьютера, на котором организуется сервис
        port - порт, который слушает сервер
        byte_order_little - порядок little-endian, если False - big-endian
        '''
        self.__ip = ip
        self.__port = port 
        self.__sock = None
        # описание буфера входящих сообщений
        self.__buf_desc = {}
        # длина буфера
        self.__buf_len = 0
        # обработчики буфера
        self.__buf_handler = {}
        # обработчики данных
        self.__data_handler = {}
        # порядок байт
        if byte_order_little:
            self.__b_order_sign = '<'
            self.__b_order = 'little'
        else:
            self.__b_order_sign = '>'
            self.__b_order = 'big'

    def run( self ):
        '''
        Запустить слушающий сервер, обрабатывать все приходящие пакеты
        '''
        self.__srv = socketserver.UDPServer\
                ( ( self.__ip, self.__port ), self.__handle_request )
        self.__srv.serve_forever()

    def add_parameter( self, name: str, offset: int, fmt: str ):
        '''
        Добавить считываемую из буфера переменную с именем name, форматом fmt
        и расположенную в буфере со смещением offset
        '''
        assert name not in self.__buf_desc
        _fmt = self.__b_order_sign + fmt
        size = struct.calcsize( _fmt )
        # проверяем пересечения областей памяти с другими параметрами
        for n,d in self.__buf_desc.items():
            # если начало текущей правее чем конец другой
            # или конец текущей левее чем начало другой
            # то все хорошо
            if not( offset >= d[1]+d[2] or offset+size <= d[1] ):
                raise DataIntersection\
                    ( 'Data intersection between variables \'{}\' and \'{}\''.\
                        format( name, n ) )
        self.__buf_desc[ name ] = ( _fmt, offset, size )
        new_len = offset + size
        if self.__buf_len < new_len:
            self.__buf_len = new_len


    def __handle_request( self, *args, **kwargs ):
        return RequestHandler( self, *args, **kwargs )

    def add_buffer_handler( self, handler: callable, priority: int=100 ):
        '''
        Добавить обработчик буфера
        '''
        if priority not in self.__buf_handler:
            self.__buf_handler[ priority ] = list()
        self.__buf_handler[ priority ].append( handler )

    def add_data_handler( self, handler: callable, priority: int=100 ):
        '''
        Добавить обработчик переменных
        '''
        if priority not in self.__data_handler:
            self.__data_handler[ priority ] = list()
        self.__data_handler[ priority ].append( handler )

    def parse_buffer( self, buf: bytes ):
        '''
        Разобрать буфер:
        считать переменные, в нужном порядке вызвать обработчики буфера
        '''
        if len( buf ) < self.__buf_len:
            raise InvalidBuffer( 'Recieved buffer with length {}, expected {}'.\
                format( len( buf ), self.__buf_len ) )
        # вызываем обработчики
        for p in sorted( self.__buf_handler.keys() ):
            for h in self.__buf_handler[ p ]:
                try:
                    h( buf )
                except Exception as e:
                    logging.warning( 'DataReceiver: buffer handler failed: {}'.\
                        format( str( e ) ) )
        param = {}
        # разбираем буфер
        for n, f in self.__buf_desc.items():
            param[ n ] = struct.unpack_from( f[0], buf, f[1] )
        logging.debug( 'DataReceiver: incoming buffer: header is {}'.\
            format( param['type'] + param['num'] + param['sum'] ) )
        # вызываем обработчики
        for p in sorted( self.__data_handler.keys() ):
            for h in self.__data_handler[ p ]:
                h( param )


if __name__ == '__main__':
    def print_vars( param ):
        print( param )
    def calc_control_sum( buf ):
        w_size = len( buf ) // 2
        r = struct.unpack_from( 'H'*w_size, buf )
        sum = 0
        for i in range( w_size ):
            if i != 1:
                sum += r[ i ]
            if sum >= 0xffff:
                sum = 1 + sum & 0xffff
        if len( buf ) % 2 != 0:
            sum += int.from_bytes( [ buf[-1], 0 ], byteorder='little', \
                    signed=False )
            if sum >= 0xffff:
                sum = 1 + sum & 0xffff
        if sum != r[1]:
            raise BufferError( 'Invalid control sum {}, expected {}'.\
                format( sum, r[1] ) )

    r = DataReceiver( 'localhost', 5253 )
    r.add_parameter( 'X', 4, 'd' )
    r.add_parameter( 'Y', 12, 'd' )
    r.add_parameter( 'Z', 20, 'd' )
    r.add_parameter( 'VX', 28, 'd' )
    r.add_parameter( 'VY', 36, 'd' )
    r.add_parameter( 'VZ', 44, 'd' )

    r.add_data_handler( print_vars )
    r.add_buffer_handler( calc_control_sum )

    buf = struct.pack( '<BBHdddddd', 1, 0, 0xf303, 0.0, 0.0, 0.0, 1.0, 0.9, 1.1 )
    r.parse_buffer( buf )

