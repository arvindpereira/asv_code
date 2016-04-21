// Definition of the ClientSocket class
// Copyright © 2002, Rob Tougher.
#ifndef ClientSocket_class
#define ClientSocket_class

#include "Socket.h"
#include <iostream>

class ClientSocket : private Socket
{
 public:

  ClientSocket ( std::string host, int port );
  virtual ~ClientSocket(){};

  const ClientSocket& operator << ( const std::string& ) const;
  const ClientSocket& operator >> ( std::string& ) const;
  int send_bytes(unsigned char *buf,int num_bytes);
  int receive_bytes (unsigned char *buf) const;
};
#endif
