/*
 * Generated by erpcgen 1.13.0 on Wed Mar 26 05:10:01 2025.
 *
 * AUTOGENERATED - DO NOT EDIT
 */


#if ERPC_ALLOCATION_POLICY == ERPC_ALLOCATION_POLICY_DYNAMIC
#include "erpc_port.h"
#endif
#include "erpc_codec.hpp"
#include "remote_display_client.hpp"
#include "erpc_manually_constructed.hpp"

#if 11300 != ERPC_VERSION_NUMBER
#error "The generated shim code version is different to the rest of eRPC code."
#endif

using namespace erpc;
using namespace std;
using namespace erpcShim;

//! @brief Function to write struct Context
static void write_Context_struct(erpc::Codec * codec, const Context * data);

//! @brief Function to write struct Command
static void write_Command_struct(erpc::Codec * codec, const Command * data);


// Write struct Context function implementation
static void write_Context_struct(erpc::Codec * codec, const Context * data)
{
    if(NULL == data)
    {
        return;
    }

    codec->write(static_cast<int32_t>(data->current_state));

    codec->write(static_cast<int32_t>(data->previous_state));

    codec->write(static_cast<int32_t>(data->mode));
}

// Write struct Command function implementation
static void write_Command_struct(erpc::Codec * codec, const Command * data)
{
    if(NULL == data)
    {
        return;
    }

    write_Context_struct(codec, &(data->context));

    codec->write(data->data);
}




DisplayInterface_client::DisplayInterface_client(ClientManager *manager)
:m_clientManager(manager)
{
}

DisplayInterface_client::~DisplayInterface_client()
{
}

// DisplayInterface interface sendDisplayCommand function client shim.
void DisplayInterface_client::sendDisplayCommand(const Command * cmd)
{
    erpc_status_t err = kErpcStatus_Success;


#if ERPC_PRE_POST_ACTION
    pre_post_action_cb preCB = m_clientManager->getPreCB();
    if (preCB)
    {
        preCB();
    }
#endif

    // Get a new request.
    RequestContext request = m_clientManager->createRequest(true);

    // Encode the request.
    Codec * codec = request.getCodec();

    if (codec == NULL)
    {
        err = kErpcStatus_MemoryError;
    }
    else
    {
        codec->startWriteMessage(message_type_t::kOnewayMessage, m_serviceId, m_sendDisplayCommandId, request.getSequence());

        write_Command_struct(codec, cmd);

        // Send message to server
        // Codec status is checked inside this function.
        m_clientManager->performRequest(request);

        err = codec->getStatus();
    }

    // Dispose of the request.
    m_clientManager->releaseRequest(request);

    // Invoke error handler callback function
    m_clientManager->callErrorHandler(err, m_sendDisplayCommandId);

#if ERPC_PRE_POST_ACTION
    pre_post_action_cb postCB = m_clientManager->getPostCB();
    if (postCB)
    {
        postCB();
    }
#endif


    return;
}
