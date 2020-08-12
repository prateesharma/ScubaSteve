####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import socket

HOST = "192.168.4.1"
PORT = 6677


class TcpSocket(object):
    """Establishes a TCP socket connection and provides functionality for
    receiving a command from the client.
    """
    def __init__(self, host=HOST, port=PORT):
        # Creates a socket object that supports the context manager type. Not
        # necessary to call s.close() so socket.socket() can be used in a with
        # statement. AF_INET specifies the address family for IPv4. SOCK_STREAM
        # is the socket type for TCP.
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # bind() is used to associate the socket with a specific network
        # interface and port number.
        self._socket.bind((host, port))

        # listen() enables a server to accept() connections making it a
        # "listening" socket.
        self._socket.listen(1)

    def listen(self):
        # accept() blocks and waits for an incoming connection. When a client
        # connects, it returns a new socket object representing the connection
        # and a tuple holding the address of the client.
        conn = None
        self._socket.settimeout(1)
        try:
            conn, addr = self._socket.accept()
            rospy.loginfo(f"Connected to {addr}")
        except socket.timeout:
            rospy.loginfo("Socket timed out. No connection established."

        # Set a timeout for the connection and receive data.
        cmd = None
        if conn:
            conn.settimeout(1)
            while True:
                try:
                    data = conn.recv(1024)
                except socket.timeout:
                    rospy.loginfo("Connection timed out. Closing connection.")
                    break

                # Extract the command and send acknowledgement.
	        if data:
                    cmd = data.decode()
                    rospy.loginfo("Command received from client: {cmd}")
                    response = "ACK: " + cmd
                    conn.sendall(str.encode(response))
                # If returned an empty bytes object, the client closed the
                # connection. As such, close the socket's connection.
                else:
                    rospy.loginfo("Client closed connection. Closing connection.")
                    conn.close()
                    break
        return cmd
