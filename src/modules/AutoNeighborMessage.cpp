#include "AutoNeighborMessage.h"
#include "MeshService.h"
#include "configuration.h"
#include "mesh/generated/meshtastic/portnums.pb.h"
#include <Arduino.h>

// Глобальный указатель для доступа из других частей прошивки
AutoNeighborMessage *autoNeighborMessage;
// Making a position logs.
AutoNeighborMessage::AutoNeighborMessage()
    : SinglePortModule("AutoNeighborMessage", meshtastic_PortNum_POSITION_APP), concurrency::OSThread("AutoNeighborMessage")
{
    // Логируем создание модуля
    LOG_INFO("AutoNeighborMessage module constructed");
}

int32_t AutoNeighborMessage::runOnce()
{
    LOG_INFO("AutoNeighborMessage runOnce started");

    auto node = nodeDB->getMeshNode(nodeDB->getNodeNum());
    if (!node || !node->has_position || node->position.latitude_i == 0) {
        LOG_WARN("No valid position in nodeDB yet. Skipping send.");
        return 30000; // Попробуем снова через 30 секунд
    }

    // 1. Выделяем пакет данных через SinglePortModule::allocDataPacket()
    // Creating a position structure for data. Initializing and making a proto info

    // Potential warning: node is a node LITE object, which is not containing any LDOP or HDOP fields.
    meshtastic_Position currPos = meshtastic_Position_init_default;
    currPos.latitude_i = node->position.latitude_i;
    currPos.longitude_i = node->position.longitude_i;
    currPos.altitude = node->position.altitude;
    currPos.time = node->position.time;
    
    
    meshtastic_MeshPacket *p = allocDataPacket();
    if (!p) {
        LOG_ERROR("allocDataPacket failed");
        return 30000; // повторим через 30 секунд
    }
    // Nanopb proto serialization
    pb_ostream_t stream = pb_ostream_from_buffer(p->decoded.payload.bytes, sizeof(p->decoded.payload.bytes));
    if(!pb_encode(&stream, meshtastic_Position_fields, &currPos)){
        LOG_ERROR("Encoding failed!");
        service->releaseToPool(p);
        return 100000;
    }


    // 4. Отправляем в Mesh-сеть
    //    service - глобальный указатель (extern MeshService *service;)
    //    Если service - объект, замените -> на .
    p->decoded.payload.size = stream.bytes_written;
    p->decoded.portnum = meshtastic_PortNum_POSITION_APP;

    service->sendToMesh(p, RX_SRC_LOCAL, false);

    

    // 5. Возвращаем интервал до следующего запуска (30 секунд)
    return 30000;
}