// Implementation of the ClientSocket class
// Copyright © 2002, Rob Tougher.
#include "ClientSocket.h"
#include "SocketException.h"


ClientSocket::ClientSocket ( std::string host, int port )
{
  if ( ! Socket::create() )
    {
      throw SocketException ( "Could not create client socket." );
    }

   if ( ! Socket::connect ( host, port ) )
    {
      throw SocketException ( "Could not bind to port." );
    }
    //Socket::set_non_blocking(false);
}


const ClientSocket& ClientSocket::operator << ( const std::string& s ) const
{
  if ( ! Socket::send ( s ) )
    {
      throw SocketException ( "Could not write to socket." );
    }

  return *this;

}

int ClientSocket::send_bytes(unsigned char *buf,int num_bytes)
{
	if(! Socket::send_bytes(buf,num_bytes))
	{
	 throw SocketException ("Could not write to socket.");
	}
	return 0;

}


const ClientSocket& ClientSocket::operator >> ( std::string& s ) const
{
  if ( ! Socket::recv ( s ) )
    {
      throw SocketException ( "Could not read from socket." );
    }

  return *this;
}



int ClientSocket::receive_bytes (unsigned char *buf) const
{
	int status=Socket::recv_bytes(buf);
  if ( status==-1 )
    {
      throw SocketException ( "Could not read from socket." );
    }

  return status;	// Number of bytes read...
}
