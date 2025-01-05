#include "recording_connection.hpp"

namespace mip
{
namespace extras
{

///@brief Creates a RecordingConnection that will write received bytes to recvStream, and sent bytes to sendStream
///
///@param connection Connection object that will actually communicate with the device
///@param recvStream The stream to write to when bytes are received. Null if received bytes should not be written to a stream
///@param sendStream The stream to write to when bytes are sent. Null if sent bytes should not be written to a stream
RecordingConnection::RecordingConnection(Connection* connection, std::ostream* recvStream, std::ostream* sendStream) :
    mConnection(connection), mRecvFile(recvStream), mSendFile(sendStream)
{
    mType = TYPE;
}

///@copydoc mip::Connection::sendToDevice
bool RecordingConnection::sendToDevice(const uint8_t* data, size_t length)
{
    const bool ok = mConnection->sendToDevice(data, length);
    if( ok && mSendFile != nullptr && mConnection->isConnected())
    {
        mSendFile->write(reinterpret_cast<const char*>(data), length);

        mSendFileWritten += length;
    }

    return ok;
}

///@copydoc mip::Connection::recvFromDevice
bool RecordingConnection::recvFromDevice(uint8_t* buffer, size_t max_length, Timeout wait_time, size_t* count_out, Timestamp* timestamp_out)
{
    const bool ok = mConnection->recvFromDevice(buffer, max_length, wait_time, count_out, timestamp_out);
    if (ok && mRecvFile != nullptr && mConnection->isConnected())
    {
        mRecvFile->write(reinterpret_cast<char*>(buffer), *count_out);

        mRecvFileWritten += *count_out;
    }

    return ok;
}

}  // namespace extras
}  // namespace mip