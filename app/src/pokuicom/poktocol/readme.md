# POKI SERIAL PROTOCOL


## POKMAC

The procotol: 
```
SIZE: | 2           | 1 |     2     | PAYLOAD_LEN | 2 |
DATA: | 0xDE | 0xAD |CMD|PAYLOAD_LEN| PAYLOAD     |CRC| 

CMDS:
    - 0: WDATA
    - 1: WDATA_NO_ACK
    - 2: ACK

```

## POKAPPCOM

```
|  1  |   1        |   X  |
| CMD | DATA_TYPE  | DATA |

CMDS:
 - 0: WRITE
 - 1: REQUEST


DATA_TYPE:
 - 0: SCORE : 1 octet
 - 1: TEAM: 1 octet [0: blue | 1: yellow]
 - 2: MATCH_STARTED: 0 octet

```

