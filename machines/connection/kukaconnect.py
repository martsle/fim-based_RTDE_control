import socket
from sys import byteorder
from machines.connection.connect import *                                                 # Used for TCP/IP communication
import struct
import time
import traceback
import os
import select

"""
    Author: Davide Rosa
    Description: Client application for KUKAVARPROXY for KRC4
    https://github.com/skilledAutomation/KUKAVARPROXY_KRC4
"""
class KUKAconnectionKRC4(RobotConnection):
    sock = None
    host = None
    port = None

    sock_timeout = 3.0
    
    #message id
    KVP_IDCOUNTER = 0 #short

    """ Byte size of the various protocol messages fields """
    KVP_IDSIZE				= 2
    KVP_LENSIZE				= 2
    KVP_IPSIZE              = 4 #i.e. 0x255 0x255 0x255 0x0
    KVP_FUNCTIONSIZE		= 1
    KVP_BLOCKSIZE			= 2
    KVP_RESULTLENGTHSIZE    = 2
    KVP_RESULTSIZE          = 1

    KVP_FUNCTION_READ		= 0
    KVP_FUNCTION_READARRAY	= 2

    KVP_FUNCTION_WRITE		= 1
    KVP_FUNCTION_WRITEARRAY	= 3

    KVP_FUNCTION_DISCOVER   = 4 #MESSAGE BODY: [1 byte FUNCTION]
                                 #REPLY MESSAGE BODY: [1 byte FUNCTION][IP ADDRESSES COUNT][4 bytes IP ADDRESS * IP ADDRESSES COUNT][RESULT LENGTH][RESULT]

    KVP_FUNCTION_SETROBOTIP = 5 #MESSAGE BODY: [1 byte FUNCTION][4 bytes IP]
                                 #REPLY MESSAGE BODY: [1 byte FUNCTION][RESULT LENGTH][RESULT]

    KVP_RESULTOK			= 1
    KVP_RESULTFAIL			= 0

    def __init__(self, _host, _port, _sockTimeout = 3.0):
        self.host: str = _host
        self.port: int = _port
        self.sock_timeout = _sockTimeout
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect()

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect( (self.host, self.port) )
            self.sock.settimeout( self.sock_timeout )
        except:
            traceback.print_exc()

    def packMessage(self, kvp_func, dataToSend):
        """ Returns the buffer ready to be sent by socket
        """
        self.KVP_IDCOUNTER = self.KVP_IDCOUNTER + 1
        if self.KVP_IDCOUNTER==0xffff:
            self.KVP_IDCOUNTER = 0

        test1 = struct.pack(">H", self.KVP_IDCOUNTER)
        test2 = self.KVP_IDCOUNTER.to_bytes(2, byteorder='big')
        _buffer = bytearray()
        _buffer.extend( struct.pack(">H", self.KVP_IDCOUNTER) ) #little endian, unsigned short (2 bytes)

        _dataLen = self.KVP_FUNCTIONSIZE + len(dataToSend)

        if _dataLen > 0xffff or _dataLen < (self.KVP_FUNCTIONSIZE):
            raise Exception()
        
        test3 = struct.pack(">H", _dataLen)
        test4 = _dataLen.to_bytes(2, byteorder='big')
        _buffer.extend( struct.pack(">H", _dataLen) ) #little endian, unsigned short (2 bytes)

        test5 = struct.pack("B", kvp_func)
        test6 = kvp_func.to_bytes(1, byteorder='big')
        _buffer.extend( struct.pack("B", kvp_func) ) #unsigned char (1 byte)

        _buffer.extend(dataToSend)
        return _buffer

    def read_message(self, data_length):
        data = bytearray()
        while (not self.sock is None) and (len(data) < data_length):
            try:
                readable, writable, errors = select.select([self.sock,], [], [], 0) #last parameter is timeout, when 0 is non blocking
                if self.sock in readable:
                    _data = self.sock.recv(data_length-len(data))
                    if len(_data) < 1:
                        return data 
                    data += _data
            except:
                #if the socket was found to be readable but the read failed, then it disconnected
                self.sock = None
                return None
        return data

    def readVar(self, varName):
        """ Returns the variable value if success otherwise None """

        if self.sock == None:
            self.connect()

        _dataToSend = bytearray()
        _dataToSend.extend( struct.pack(">H", len(varName)))
        _dataToSend.extend(varName.encode("utf-8"))
        _msg = self.packMessage(self.KVP_FUNCTION_READ, _dataToSend)

        try:
            if self.sock.send(_msg) == len(_msg):
            
                _reply = self.read_message(4) #msg_id + msg_size
                _msgID, _msgSize = struct.unpack(">HH", _reply)

                if (not _msgID == self.KVP_IDCOUNTER) or (_msgSize < (self.KVP_FUNCTIONSIZE + self.KVP_BLOCKSIZE*2)):
                    print("readVar - recv bad response length")
                else:
                    _reply = self.read_message(_msgSize)
                    
                    if not _reply[0] == self.KVP_FUNCTION_READ:
                        print("readVar - invalid packet, the returned function doesn't match") 
                    else:
                        _reply = _reply[1:] #removing the first byte that is the function

                        _varValueSize = struct.unpack(">H", _reply[0:2])[0]
                        _reply = _reply[2:] #removing the variable value size

                        _varValue = struct.unpack("%ss"%_varValueSize, _reply[0:_varValueSize])[0]
                        _reply = _reply[_varValueSize:] #removing variable value

                        #this is not useful, the result size is 1, but I leave this as is
                        _resultSize = struct.unpack(">H", _reply[0:2])[0]
                        _reply = _reply[2:] #removing the result size

                        result = _reply[0]

                        if result == self.KVP_RESULTOK:
                            return _varValue
                        else:
                            print("readVar - result not OK")

            self.sock.close()
        except:
            print("readVar - exception, varname: %s"%varName)
            traceback.print_exc()
        
        self.sock = None
        return None

    def readArray(self, varName):
        """ varName (str): the variable name with [] at the end. i.e. MYARRAY[]
            Returns the array of shorts (2 bytes) if success otherwise None """

        if self.sock == None:
            self.connect()

        _dataToSend = bytearray()
        _dataToSend.extend( struct.pack(">H", len(varName)))
        _dataToSend.extend(varName.encode("utf-8"))
        _msg = self.packMessage(self.KVP_FUNCTION_READARRAY, _dataToSend)

        if self.sock.send(_msg) == len(_msg):
            try:
                _reply = self.read_message(4) #msg_id + msg_size
                _msgID, _msgSize = struct.unpack(">HH", _reply)

                if (not _msgID == self.KVP_IDCOUNTER) or (_msgSize < (self.KVP_FUNCTIONSIZE + self.KVP_BLOCKSIZE*2)):
                    print("readArray - recv bad response length")
                else:
                    _reply = self.read_message(_msgSize)
                    
                    if not _reply[0] == self.KVP_FUNCTION_READARRAY:
                        print("readArray - invalid packet, the returned function doesn't match") 
                    else:
                        _reply = _reply[1:] #removing the first byte that is the function

                        _varValueSize = struct.unpack(">H", _reply[0:2])[0]
                        _reply = _reply[2:] #removing the array size in bytes

                        _varValues = struct.unpack(">" + "H"*int(_varValueSize/2),_reply[0:_varValueSize])
                        _reply = _reply[_varValueSize:] #removing variable value

                        #this is not useful, the result size is 1, but I leave this as is
                        _resultSize = struct.unpack(">H", _reply[0:2])[0]
                        _reply = _reply[2:] #removing the result size

                        result = _reply[0]

                        if result == self.KVP_RESULTOK:
                            return _varValues
                        else:
                            print("readArray - result not OK")
                
                self.sock.close()
            except:
                print("readArray - exception")
                traceback.print_exc()
                
        self.sock = None
        return None

    def writeVar(self, varName, varValue, debug):
        """ Returns True if success """

        if (self.sock == None) and (not debug):
            self.connect()

        if len(varName) > 0xffff or len(varValue) > 0xffff:
            print("writeVar - var name or value too long")

        _dataToSend = bytearray()
        _dataToSend.extend( struct.pack(">H", len(varName)))
        _dataToSend.extend(varName.encode("utf-8"))
        _dataToSend.extend( struct.pack(">H", len(varValue)))
        _dataToSend.extend(varValue.encode("utf-8"))
        _msg = self.packMessage(self.KVP_FUNCTION_WRITE, _dataToSend)
        
        try:
            if debug:
                 print(_msg)
                 return
            
            elif self.sock.send(_msg) == len(_msg):
            
                _reply = self.read_message(4) #msg_id + msg_size
                _msgID, _msgSize = struct.unpack(">HH", _reply)

                if (not _msgID == self.KVP_IDCOUNTER) or (_msgSize < (self.KVP_FUNCTIONSIZE + self.KVP_BLOCKSIZE*2)):
                    print("writeVar - recv bad response length")
                else:
                    _reply = self.read_message(_msgSize)
                    
                    if not _reply[0] == self.KVP_FUNCTION_WRITE:
                        print("writeVar - invalid packet, the returned function doesn't match") 
                    else:
                        _reply = _reply[1:] #removing the first byte that is the function

                        _varValueSize = struct.unpack(">H", _reply[0:2])[0]
                        _reply = _reply[2:] #removing the variable value size

                        _varValue = struct.unpack("%ss"%_varValueSize, _reply[0:_varValueSize])[0]
                        _reply = _reply[_varValueSize:] #removing variable value

                        #this is not useful, the result size is 1, but I leave this as is
                        _resultSize = struct.unpack(">H", _reply[0:2])[0]
                        _reply = _reply[2:] #removing the result size

                        result = _reply[0]

                        if result == self.KVP_RESULTOK:
                            return True
                        else:
                            print("writeVar - result not OK")
            self.sock.close()
        except:
            print("writeVar - exception, varname: %s"%varName)
            traceback.print_exc()
                
        self.sock = None
        return False

    def writeArray(self, varName, varValues):
        """ varName (str): the variable name with [] at the end. i.e. MYARRAY[]
            varValues (list of shorts (2bytes)): the array values
            Returns True if success 
        
        """

        if self.sock == None:
            self.connect()

        if len(varName) > 0xffff or len(varValues) > 0xffff:
            print("writeVar - var name or value too long")

        _dataToSend = bytearray()
        _dataToSend.extend( struct.pack(">H", len(varName)))
        _dataToSend.extend(varName.encode("utf-8"))
        _dataToSend.extend( struct.pack(">H", len(varValues)*2))
        _dataToSend.extend(struct.pack(">%sH"%len(varValues),*varValues))
        _msg = self.packMessage(self.KVP_FUNCTION_WRITEARRAY, _dataToSend)
        
        if self.sock.send(_msg) == len(_msg):
            try:
                _reply = self.read_message(4) #msg_id + msg_size
                _msgID, _msgSize = struct.unpack(">HH", _reply)

                if (not _msgID == self.KVP_IDCOUNTER) or (_msgSize < (self.KVP_FUNCTIONSIZE + self.KVP_BLOCKSIZE*2)):
                    print("writeArray - recv bad response length")
                else:
                    _reply = self.read_message(_msgSize)
                    
                    if not _reply[0] == self.KVP_FUNCTION_WRITEARRAY:
                        print("writeArray - invalid packet, the returned function doesn't match") 
                    else:
                        _reply = _reply[1:] #removing the first byte that is the function

                        _varNameSize = struct.unpack(">H", _reply[0:2])[0]
                        _reply = _reply[2:] #removing the variable name size

                        _varValue = struct.unpack("%ds"%_varNameSize, _reply[0:_varNameSize])[0]
                        _reply = _reply[_varNameSize:] #removing variable value

                        #this is not useful, the result size is 1, but I leave this as is
                        _resultSize = struct.unpack(">H", _reply[0:2])[0]
                        _reply = _reply[2:] #removing the result size

                        result = _reply[0]

                        if result == self.KVP_RESULTOK:
                            return True
                        else:
                            print("writeArray - result not OK")
                self.sock.close()
            except:
                print("writeArray - exception")
                traceback.print_exc()
                
        self.sock = None
        return False

    def parseStructure(self, value):
        """ Given a variable value of type struct, 
            this function returns a dictionary three of the parsed value

            value (str): the kuka string representation of a struct 

            Returns a dictionary
        """
        resultDict = {}
        value = value[value.index(':'):]
        value = value.replace('{','').replace(' ','').replace('}','')
        fields = value.split(',')
        for v in fields:
            v = v.strip()
            fieldName = v[:v.index(' ')]
            fieldValue = v[v.index(' ')+1:]
            resultDict[fieldName] = fieldValue

            if fieldValue.startswith('{'):
                resultDict[fieldName] = self.parseStructure(fieldValue)
        return resultDict

    def packStructure(self, structTypeName, valuesDict):
        ret = '{%s:'%structTypeName
        for fieldName, fieldValue in valuesDict.items():
            if type(fieldValue) == dict:
                fieldValue = self.parseStructure(fieldName, fieldValue)
            ret = ret + " " + fieldName + " " + str(fieldValue) + ","

        return ret

    def discoverRobots(self):
        """ Returns the IPs of the available robots  """
        ipList = []

        if self.sock == None:
            self.connect()

        #MESSAGE BODY: [1 byte FUNCTION]
        _dataToSend = bytearray()
        data = [int(s) for s in self.host.split('.')]
        _dataToSend.extend(struct.pack(">BBBB", *data))
        _msg = self.packMessage(self.KVP_FUNCTION_DISCOVER, _dataToSend)

        try:
            if self.sock.send(_msg) == len(_msg):
                #REPLY MESSAGE BODY: [1 byte FUNCTION][2 bytes IP ADDRESSES COUNT][4 bytes IP ADDRESS * IP ADDRESSES COUNT][RESULT LENGTH][RESULT]
                _reply = self.read_message(4) #msg_id + msg_size
                _msgID, _msgSize = struct.unpack(">HH", _reply)

                if (not _msgID == self.KVP_IDCOUNTER) or (_msgSize < (self.KVP_FUNCTIONSIZE + self.KVP_BLOCKSIZE*2 + 1)):
                    print("readVar - recv bad response length")
                else:
                    _reply = self.read_message(_msgSize)
                    
                    if not _reply[0] == self.KVP_FUNCTION_DISCOVER:
                        print("readVar - invalid packet, the returned function doesn't match") 
                    else:
                        _reply = _reply[1:] #removing the first byte that is the function

                        _ipAddressCount = struct.unpack(">H", _reply[0:2])[0]
                        _reply = _reply[2:] #removing the variable value size

                        for ip_index in range(0, _ipAddressCount):
                            ip = _reply[0:4]
                            ipList.append(ip)
                            _reply = _reply[4:] #removing variable value

                        #this is not useful, the result size is 1, but I leave this as is
                        _resultSize = struct.unpack(">H", _reply[0:2])[0]
                        _reply = _reply[2:] #removing the result size

                        result = _reply[0]

                        if result == self.KVP_RESULTOK:
                            return ipList
                        else:
                            print("readVar - result not OK")

            self.sock.close()
        except:
            traceback.print_exc()
        
        self.sock = None
        return []

    def setRobotIP(self, ip):
        """ Sets the ip of the server robot 
            Args:
                ip (list) = list of 4 ip bytes
        """
        #MESSAGE BODY: [1 byte FUNCTION][4 bytes IP]

        if self.sock == None:
            self.connect()

        _dataToSend = bytearray(ip)
        _msg = self.packMessage(self.KVP_FUNCTION_SETROBOTIP, _dataToSend)

        try:
            if self.sock.send(_msg) == len(_msg):
                #REPLY MESSAGE BODY: [1 byte FUNCTION][RESULT LENGTH][RESULT]
                _reply = self.read_message(4) #msg_id + msg_size
                _msgID, _msgSize = struct.unpack(">HH", _reply)

                if (not _msgID == self.KVP_IDCOUNTER) or (_msgSize < (self.KVP_FUNCTIONSIZE + self.KVP_BLOCKSIZE + 1)):
                    print("readVar - recv bad response length")
                else:
                    _reply = self.read_message(_msgSize)
                    
                    if not _reply[0] == self.KVP_FUNCTION_SETROBOTIP:
                        print("readVar - invalid packet, the returned function doesn't match") 
                    else:
                        _reply = _reply[1:] #removing the first byte that is the function

                        #this is not useful, the result size is 1, but I leave this as is
                        _resultSize = struct.unpack(">H", _reply[0:2])[0]
                        _reply = _reply[2:] #removing the result size

                        result = _reply[0]

                        if result == self.KVP_RESULTOK:
                            return True
                        else:
                            print("readVar - result not OK")

            self.sock.close()
        except:
            traceback.print_exc()
        
        self.sock = None
        return False

    def send(self, content, debug:bool = False) -> None:
        self.writeVar('$ROBO_Command', str(content), debug)

    def send_init_package(self, content, debug:bool = False) -> None:
        self.writeVar('$INIT', str(content), debug)
        self.writeVar('$INIT_Send', str(1), debug)


    def receive(self):
        return self.readVar('$POS_ACT')

    def disconnect(self) -> None:
        self.sock.close()


class KUKAconnectionKRC2(RobotConnection):

	def __init__(self, TCP_IP, PORT, testmode = False):
		super().__init__(TCP_IP, PORT)
		if not testmode:
			client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)		# Initializing client connection
			try: 
				client.connect((TCP_IP, PORT))                      # Open socket. kukavarproxy actively listens on TCP port 7000
			except: 
				self.error_list(1)

	def send (self, var, val, msgID, testmode = False):
		"""
		kukavarproxy message format is 
		msg ID in HEX                       2 bytes
		msg length in HEX                   2 bytes
		read (0) or write (1)               1 byte
		variable name length in HEX         2 bytes
		variable name in ASCII              # bytes
		variable value length in HEX        2 bytes
		variable value in ASCII             # bytes
		"""
		try:
			msg = bytearray()
			temp = bytearray()
			if val != "":
				val = str(val)
				msg.append((len(val) & 0xff00) >> 8)            # MSB of variable value length
				msg.append((len(val) & 0x00ff))                 # LSB of variable value length
				msg.extend(map(ord, val))                       # Variable value in ASCII
			temp.append(bool(val))                              # Read (0) or Write (1)
			temp.append(((len(var)) & 0xff00) >> 8)             # MSB of variable name length
			temp.append((len(var)) & 0x00ff)                    # LSB of variable name length
			temp.extend(map(ord, var))                          # Variable name in ASCII 
			msg = temp + msg
			del temp[:]
			temp.append((msgID & 0xff00) >> 8)                  # MSB of message ID
			temp.append(msgID & 0x00ff)                         # LSB of message ID
			temp.append((len(msg) & 0xff00) >> 8)               # MSB of message length
			temp.append((len(msg) & 0x00ff))                    # LSB of message length
			msg = temp + msg
		except :
			self.error_list(2)
		try:
			if not testmode:
				self.client.send(msg)
				return  self.client.recv(1024)                           # Return response with buffer size of 1024 bytes
			else:
				return msg
		except :
			self.error_list(1)

	def __get_var(self, msg):
		"""
		kukavarproxy response format is 
		msg ID in HEX                       2 bytes
		msg length in HEX                   2 bytes
		read (0) or write (1)               1 byte
		variable value length in HEX        2 bytes
		variable value in ASCII             # bytes
		Not sure if the following bytes contain the client number, or they're just check bytes. I'll check later.
		"""
		try:
			'''
			# Python 2.x
			lsb = (int (str(msg[5]).encode('hex'),16))
			msb = (int (str(msg[6]).encode('hex'),16))
			lenValue = (lsb <<8 | msb)
			return msg [7: 7+lenValue]
			'''
			
			# Python 3.x
			lsb = int( msg[5])
			msb = int( msg[6])
			lenValue = (lsb <<8 | msb)
			return str(msg [7: 7+lenValue],'utf-8')  
			

		except:
			self.error_list(2)

	def read (self, var, msgID=0):
		
		try:
			ans =  self.__get_var(self.send(var,"",msgID))  
			return State(0,0,0) # TBD
		except :
			self.error_list(2)

	def write (self, var, val, msgID=0):
		# Packen der Nachricht



		#
		try:
			if val != (""): return self.__get_var(self.send(var,val,msgID))
			else: raise self.error_list(3)
		except :
			self.error_list(2)

	def disconnect (self):
			self.client.close()                                      # CLose socket

	def error_list (self, ID):
		if ID == 1:
			print ("Network Error (tcp_error)")
			print ("    Check your KRC's IP address on the network, and make sure kukaproxyvar is running.")
			self.disconnect()
			raise SystemExit
		elif ID == 2:
			print ("Python Error.")
			print ("    Check the code and uncomment the lines related to your python version.")
			self.disconnect()
			raise SystemExit
		elif ID == 3:
			print ("Error in write() statement.")
			print ("    Variable value is not defined.")

class State(object):
	def __init__(self, joints, pos, status):
		self.joints = joints
		self.pos = pos
		self.status = status



