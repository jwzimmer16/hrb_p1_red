RB_NMT = 0x00
RB_COMMAND = 0x0100
RB_SENSOR = 0x0200
RB_USART = 0X0300
RB_NEIGHBOR = 0X0400
RB_DICT_RESP = 0x0500        
RB_DICT_REQ = 0x0600
RB_HEARTBEAT = 0x0700    


RB_CS_READ = 0x40
RB_CS_WRITE = 0x20

RB_CS_START = 0x01
RB_CS_STOP = 0x02
RB_CS_RESET = 129

permissions = {0x01: 'Read Only', 
               0x02: 'Write Only', 
               0x03: 'Read Write'} 

RB_PERM_READ = 0x01
RB_PERM_WRITE = 0x02

types = {0x41: 'BOOLEAN',
         0x42: 'SINT8',
         0x43: 'SINT16',
         0x44: 'SINT32',
         0x45: 'UINT8',
         0x46: 'UINT16',
         0x47: 'UINT32',
         0x48: 'FLOAT32',
         0x49: 'STRING32',
         0x51: 'FLOAT64',
         0x55: 'SINT64',
         0x5B: 'UINT64',
         0x60: 'PM_CONFIG',
         0x61: 'PM_MAPPING',
         0x63: 'IDENTITY'}

for k,v in types.iteritems():
  exec('RB_%s = %d' % (v,k))
  
DTYPES = { 0x41 : (bool,'B','bool',1), 
           0x42 : (int,'b','sint8',1), 
           0x43 : (int,'h','sint16',2),
           0x44 : (int, 'i', 'sint32',4),
           0x45 : (int,'B','uint8',1), 
           0x46 : (int,'H','uint16',2),
           0x47 : (long, 'I', 'uint32',4),
           0x48 : (float, 'f', 'float32',4),
           0x49 : (str, 's', 'string32',32),
           0x50 : (float, 'f', 'float64',8),
           0x55 : (long, 'I', 'sint64',8),
           0x5B : (long,'I','uint64',8),
           0x60 : (int, 'I', 'pm_config',4),
           0x61 : (long, 'I', 'pm_mapping',1013),
           0x63 : (int, 'B', 'identity',1)
           }

def decode_boolean(data):
    pass

def decode_sint8(data):
    pass

def decode_sint16(data):
    pass

decode = {'BOOLEAN': decode_boolean,
          'SINT8': decode_sint8,
          'SINT16': decode_sint16}
#ETC.
