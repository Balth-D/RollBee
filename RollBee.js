const {deviceEndpoints, windowCovering, temperature, humidity} = require('zigbee-herdsman-converters/lib/modernExtend');

const definition = {
    zigbeeModel: ['RollBee'],
    model: 'RollBee',
    vendor: 'Balth.D',
    description: 'RollBee covering device',
    extend: [
        deviceEndpoints({"endpoints":{"1":1, "10":10}}),
        windowCovering({"controls":["lift"]}),
        temperature({"endpointNames":["10"]}),
        humidity({"endpointNames":["10"]})
    ],
    meta: {"multiEndpoint":true},
};

module.exports = definition;
