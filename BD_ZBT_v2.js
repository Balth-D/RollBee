const {deviceEndpoints, windowCovering, temperature, humidity} = require('zigbee-herdsman-converters/lib/modernExtend');

const definition = {
    zigbeeModel: ['BD_ZBT_C_v2'],
    model: 'ZB_Things Covering v2',
    vendor: 'Balth.D',
    description: 'ZB_Things covering device',
    extend: [deviceEndpoints({"endpoints":{"1":1, "10":10}}), windowCovering({"controls":["lift"]}), temperature({"endpointNames":["10"]}), humidity({"endpointNames":["10"]})],
    meta: {"multiEndpoint":true},
};

module.exports = definition;
