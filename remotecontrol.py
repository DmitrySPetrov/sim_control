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
Модуль, в котором реализовано удаленное управление программой
1. Осуществляет прием команд и отправляет ответы
2. Отсылает значения переменных
3. Отсылает диагностические строки
'''

import socketserver
import queue
import struct
import logging
import datetime


def date2numbers( dt ):
    '''
    Превращаем дату в два числа: грегорианскую дату и время от начала дня
    '''
    date = dt.date()
    t = ( dt - date ).total_seconds()
    return date.toordinal(), t


class RequestHandler( socketserver.BaseRequestHandler ):
    def __init__( self, rcv_class, *args, **kwargs ):
        self.__rcv_class = rcv_class
        super().__init__( *args, **kwargs )

    def handle( self ):
        data = self.request[0]
        self.__rcv_class.parse_buffer( data )

class RemoteControl:

    # Время ожидания в секундах при проведении операций с очередью
    QUEUE_TIMEOUT = 1e-4

    def __init__( self, cmd_host: str, cmd_port: int, \
            ctrl_host: str, msgs_port: int, vars_port: int ):
        '''
        cmd_host - адрес этого компьютера
        cmd_port - порт, на который приходят команды
        ctrl_host - адрес, по которому доступна программа удаленного управления
        msgs_port - порт, на который отсылаем диагностические сообщения
        vars_port - порт, на который отсылаем переменные
        '''
        self.__srv_addr = ( cmd_host, cmd_port )
        self.__srv = None
        self.__q_cmd = queue.Queue()

        self.__msgs_addr = ( ctrl_host, msgs_port )
        self.__q_msg = queue.Queue()
        self.__vars_addr = ( ctrl_host, vars_port )
        self.__q_vars = queue.Queue()

        self.__td = None

    def run_server( self ):
        self.__srv = socketserver.UDPServer\
                ( self.__srv_addr, self.__handle_request )
        self.__srv.serve_forever()

    def __handle_request( self, *args, **kwargs ):
        return RequestHandler( self, *args, **kwargs )

    # Тип: сообщения от ПО управления имитатора АСН
    MSGS_DATA_TYPE = 24853

    def send_messages( self ):
        try:
            if not self.__q_msg.empty():
                msg = self.__q_msg.get_nowait()

                # Превращаем дату в два числа: грегорианскую дату и время
                # от начала дня
                head = struct.pack( 'Id', date2numbers( msg[0] ) )

                # Структура пакета
                # <тип, 2б><суммарная длина, 2б><сообщение>
                buf = head + msg[1].encode( 'utf8' )
                buf = struct.pack( 'HH', self.MSGS_DATA_TYPE, len( buf ) ) \
                        + buf
                s = socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
                s.sendto( buf, self.__msgs_addr )

        except queue.Empty():
            pass

    def add_msg( self, msg, time=None ):
        if time == None:
            time = datetime.datetime.now()
        self.__q_msg.put( ( time, msg ), timeout=self.QUEUE_TIMEOUT )

    # Тип: переменные ПО управления имитатора АСН
    VARS_DATA_TYPE = 40395

    def send_variables( self ):
        try:
            if not self.__q_vars.empty():
                v = self.__q_vars.get_nowait()

                # Превращаем дату в два числа: грегорианскую дату и время
                # от начала дня
                head = struct.pack( 'Id', date2numbers( msg[0] ) )

                # Структура пакета
                # <тип, 2б><суммарная длина, 2б><массив в JSON>
                buf = head + json.dumps( msg[1] ).encode( 'utf8' )
                buf = struct.pack( 'HH', self.VARS_DATA_TYPE, len( buf ) ) \
                        + buf
                s = socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
                s.sendto( buf, self.__vars_addr )

        except queue.Empty():
            pass

    def add_variables( self, v, time=None ):
        if time == None:
            time = datetime.datetime.now()
        self.__q_vars.put( ( time, v ), timeout=self.QUEUE_TIMEOUT )


    # Тип сообщений: команды для управления ПО управления имитатора АСН
    REMOTE_CONTROL_COMMAND_TYPE = 15125

    def parse_buffer( self, data ):
        # Структура входящего пакета:
        # <тип, 2б><длина, 2б><строка>
        # строка может состоять из нескольких команд, разделенных символом ";"
        t = datetime.datetime.now()
        type_,len_ = struct.unpack_from( 'HH', data )
        if type_ != self.REMOTE_CONTROL_COMMAND_TYPE:
            logging.warning( 'RemoteControl: invalid buffer type: {}'.\
                    format( type_ ) )
        if len_ != len( data[4:] ):
            logging.warning( 'RemoteControl: invalid string length: {}, got: {}'.\
                    format( len_, len( data[4:] ) ) )
        try:
            cmd_line = data[4:].decode( 'utf8' )
        except UnicodeDecodeError:
            logging.warning( 'RemoteControl: invalid command string: {}'.\
                    format( data[4:] ) )
            return

        commands = [ c.strip() for c in cmd_line.split( ';' ) ]

        #self.__q_cmd.put( ( t, command, ), timeout=self.QUEUE_TIMEOUT )

        for cmd in commands:
            logging.debug( 'RemoteControl: got remote command: {}'.\
                    format( cmd ) )
            self.process_command( t, cmd )

    def process_command( self, time, cmd ):
        if cmd == 'halt' or cmd == 'краб':
            if self.__td:
                logging.info( 'RemoteControl: got HALT command' )
                self.__td.stop()

    @property
    def commands( self ):
        res = []
        try:
            while not self.__q_cmd.empty():
                res.append( self.__q_cmd.get_nowait() )
        except queue.Empty:
            pass
        return res

    def bind_task_dispatcher( self, td ):
        self.__td = td


