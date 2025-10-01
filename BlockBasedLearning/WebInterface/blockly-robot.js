// bktm auto blockly blocks
window.RobotGenerators = {};

function defineRobotBlocks() {
    Blockly.Blocks['robot_move_forward'] = {
        init: function() {
            this.appendDummyInput()
                .appendField("🚗 Vorwärts")
                .appendField(new Blockly.FieldDropdown([
                    ["langsam (100)", "100"],
                    ["mittel (150)", "150"],
                    ["schnell (200)", "200"]
                ]), "SPEED")
                .appendField("für")
                .appendField(new Blockly.FieldDropdown([
                    ["0,5 Sek", "0.5"],
                    ["1 Sek", "1"],
                    ["2 Sek", "2"],
                    ["3 Sek", "3"]
                ]), "DURATION");
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(120);
            this.setTooltip("Fährt vorwärts mit gewählter Geschwindigkeit");
        }
    };

    Blockly.Blocks['robot_move_backward'] = {
        init: function() {
            this.appendDummyInput()
                .appendField("🚗 Rückwärts")
                .appendField(new Blockly.FieldDropdown([
                    ["langsam (100)", "100"],
                    ["mittel (150)", "150"],
                    ["schnell (200)", "200"]
                ]), "SPEED")
                .appendField("für")
                .appendField(new Blockly.FieldDropdown([
                    ["0,5 Sek", "0.5"],
                    ["1 Sek", "1"],
                    ["2 Sek", "2"],
                    ["3 Sek", "3"]
                ]), "DURATION");
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(120);
            this.setTooltip("Fährt rückwärts mit gewählter Geschwindigkeit");
        }
    };

    Blockly.Blocks['robot_turn_degrees'] = {
        init: function() {
            this.appendDummyInput()
                .appendField("🔄 Drehe")
                .appendField(new Blockly.FieldDropdown([
                    ["90°", "90"],
                    ["180°", "180"],
                    ["360°", "360"],
                    ["45°", "45"]
                ]), "DEGREES")
                .appendField("nach")
                .appendField(new Blockly.FieldDropdown([
                    ["links", "LEFT"],
                    ["rechts", "RIGHT"]
                ]), "DIRECTION");
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(120);
            this.setTooltip("Dreht um eine bestimmte Gradzahl");
        }
    };

    Blockly.Blocks['robot_stop'] = {
        init: function() {
            this.appendDummyInput()
                .appendField("🛑 STOPP");
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(0);
            this.setTooltip("Stoppt alle Motoren sofort");
        }
    };

    Blockly.Blocks['robot_servo'] = {
        init: function() {
            this.appendDummyInput()
                .appendField("📷 Servo auf")
                .appendField(new Blockly.FieldDropdown([
                    ["links (45°)", "45"],
                    ["mitte (90°)", "90"],
                    ["rechts (135°)", "135"]
                ]), "ANGLE");
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(230);
            this.setTooltip("Dreht den Servo auf eine Position");
        }
    };

    Blockly.Blocks['robot_get_distance'] = {
        init: function() {
            this.appendDummyInput()
                .appendField("📏 Abstand (cm)");
            this.setOutput(true, "Number");
            this.setColour(230);
            this.setTooltip("Gibt den Abstand zum Hindernis zurück");
        }
    };

    Blockly.Blocks['robot_servo_scan'] = {
        init: function() {
            this.appendDummyInput()
                .appendField("🔍 Scanne Umgebung");
            this.appendDummyInput()
                .appendField("Wenn überall frei (>")
                .appendField(new Blockly.FieldDropdown([
                    ["20cm", "20"],
                    ["30cm", "30"],
                    ["40cm", "40"],
                    ["50cm", "50"]
                ]), "DISTANCE")
                .appendField(")");
            this.appendStatementInput("DO")
                .appendField("dann");
            this.appendStatementInput("ELSE")
                .appendField("sonst (Hindernis gefunden)");
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(230);
            this.setTooltip("Scannt mit Servo in alle Richtungen und prüft ob überall genug Platz ist");
        }
    };

    Blockly.Blocks['robot_find_best_direction'] = {
        init: function() {
            this.appendDummyInput()
                .appendField("🧭 Finde beste Richtung");
            this.setOutput(true, "String");
            this.setColour(230);
            this.setTooltip("Scannt alle Richtungen und gibt zurück wo am meisten Platz ist (links/mitte/rechts)");
        }
    };

    Blockly.Blocks['robot_calibrate_turns'] = {
        init: function() {
            this.appendDummyInput()
                .appendField("🎯 Kalibriere Drehungen")
                .appendField("Geschwindigkeit:")
                .appendField(new Blockly.FieldDropdown([
                    ["100", "100"],
                    ["150", "150"],
                    ["200", "200"]
                ]), "SPEED");
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(290);
            this.setTooltip("Führt eine Kalibrierungsroutine für präzise Drehungen aus");
            this.setHelpUrl("");
        }
    };

    Blockly.Blocks['robot_if_obstacle'] = {
        init: function() {
            this.appendDummyInput()
                .appendField("Wenn Hindernis näher als")
                .appendField(new Blockly.FieldDropdown([
                    ["5cm", "5"],
                    ["10cm", "10"],
                    ["15cm", "15"],
                    ["20cm", "20"],
                    ["30cm", "30"],
                    ["50cm", "50"]
                ]), "DISTANCE");
            this.appendStatementInput("DO")
                .appendField("dann");
            this.appendStatementInput("ELSE")
                .appendField("sonst");
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(210);
            this.setTooltip("Führt Aktionen aus basierend auf Hindernis-Distanz");
        }
    };

    Blockly.Blocks['robot_repeat'] = {
        init: function() {
            this.appendDummyInput()
                .appendField("🔁 Wiederhole")
                .appendField(new Blockly.FieldDropdown([
                    ["2 mal", "2"],
                    ["3 mal", "3"],
                    ["5 mal", "5"],
                    ["10 mal", "10"]
                ]), "TIMES");
            this.appendStatementInput("DO")
                .appendField("mache");
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(120);
            this.setTooltip("Wiederholt Aktionen mehrmals");
        }
    };

    Blockly.Blocks['robot_forever'] = {
        init: function() {
            this.appendDummyInput()
                .appendField("♾️ Wiederhole endlos");
            this.appendStatementInput("DO")
                .appendField("mache");
            this.setPreviousStatement(true, null);
            this.setColour(120);
            this.setTooltip("Wiederholt Aktionen endlos bis zum Stopp");
        }
    };

    Blockly.Blocks['robot_wait'] = {
        init: function() {
            this.appendDummyInput()
                .appendField("⏱️ Warte")
                .appendField(new Blockly.FieldDropdown([
                    ["0,5 Sek", "0.5"],
                    ["1 Sek", "1"],
                    ["2 Sek", "2"]
                ]), "DURATION");
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(290);
            this.setTooltip("Wartet die angegebene Zeit");
        }
    };

    Blockly.Blocks['robot_print'] = {
        init: function() {
            this.appendDummyInput()
                .appendField("📝 Zeige:")
                .appendField(new Blockly.FieldTextInput("Hallo!"), "TEXT");
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(160);
            this.setTooltip("Zeigt Text in der Konsole");
        }
    };

    Blockly.Blocks['robot_print_distance'] = {
        init: function() {
            this.appendValueInput("DISTANCE")
                .setCheck("Number")
                .appendField("📏 Zeige Abstand:");
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(160);
            this.setTooltip("Zeigt den gemessenen Abstand");
        }
    };
}

function getStatementCode(block, name) {
    let targetBlock = block.getInputTargetBlock(name);
    let code = '';
    while (targetBlock) {
        code += generateBlockCode(targetBlock);
        targetBlock = targetBlock.getNextBlock();
    }
    return code;
}

function getValueCode(block, name) {
    const targetBlock = block.getInputTargetBlock(name);
    if (!targetBlock) return ['0', 0];

    const generator = window.RobotGenerators[targetBlock.type];
    if (!generator) return ['0', 0];

    const result = generator(targetBlock);
    return Array.isArray(result) ? result : [result, 0];
}

function defineRobotGenerators() {
    window.RobotGenerators['robot_move_forward'] = function(block) {
        const speed = block.getFieldValue('SPEED');
        const duration = block.getFieldValue('DURATION');
        return 'await moveRobot(' + speed + ', ' + speed + ', ' + duration + ' * 1000);\n';
    };

    window.RobotGenerators['robot_move_backward'] = function(block) {
        const speed = block.getFieldValue('SPEED');
        const duration = block.getFieldValue('DURATION');
        return 'await moveRobot(-' + speed + ', -' + speed + ', ' + duration + ' * 1000);\n';
    };

    window.RobotGenerators['robot_turn_degrees'] = function(block) {
        const direction = block.getFieldValue('DIRECTION');
        const degrees = block.getFieldValue('DEGREES');
        const speed = '150';

        if (direction === 'LEFT') {
            return 'await turnDegrees("left", ' + degrees + ', ' + speed + ');\n';
        } else {
            return 'await turnDegrees("right", ' + degrees + ', ' + speed + ');\n';
        }
    };

    window.RobotGenerators['robot_stop'] = function(block) {
        return 'await stopRobot();\n';
    };

    window.RobotGenerators['robot_wait'] = function(block) {
        const duration = block.getFieldValue('DURATION');
        return 'await wait(' + duration + ' * 1000);\n';
    };

    window.RobotGenerators['robot_servo'] = function(block) {
        const angle = block.getFieldValue('ANGLE');
        return 'await setServo(' + angle + ');\n';
    };

    window.RobotGenerators['robot_get_distance'] = function(block) {
        return ['(await getDistance())', 0];
    };

    window.RobotGenerators['robot_if_obstacle'] = function(block) {
        const distance = block.getFieldValue('DISTANCE');
        let branch = getStatementCode(block, 'DO');
        let elseBranch = getStatementCode(block, 'ELSE');

        let code = 'if ((await getDistance()) < ' + distance + ') {\n';
        code += branch;
        if (elseBranch) {
            code += '} else {\n';
            code += elseBranch;
        }
        code += '}\n';
        return code;
    };

    window.RobotGenerators['robot_repeat'] = function(block) {
        const times = block.getFieldValue('TIMES');
        let branch = getStatementCode(block, 'DO');
        return 'for (let i = 0; i < ' + times + '; i++) {\n' + branch + '}\n';
    };

    window.RobotGenerators['robot_forever'] = function(block) {
        let branch = getStatementCode(block, 'DO');
        return 'while (!window.stopRequested) {\n' + branch + 'await wait(10);\n}\n';
    };

    window.RobotGenerators['robot_print'] = function(block) {
        const text = block.getFieldValue('TEXT');
        return 'print("' + text + '");\n';
    };

    window.RobotGenerators['robot_print_distance'] = function(block) {
        const distanceCode = getValueCode(block, 'DISTANCE');
        return 'print("Abstand: " + ' + distanceCode[0] + ' + " cm");\n';
    };

    // Servo Scan Generator
    window.RobotGenerators['robot_servo_scan'] = function(block) {
        const minDistance = block.getFieldValue('DISTANCE');
        let thenBranch = getStatementCode(block, 'DO');
        let elseBranch = getStatementCode(block, 'ELSE');

        let code = '';
        code += 'const scanResults = await performServoScan();\n';
        code += 'const allClear = scanResults.every(result => result.distance > ' + minDistance + ');\n';
        code += 'if (allClear) {\n';
        code += thenBranch;
        if (elseBranch) {
            code += '} else {\n';
            code += elseBranch;
        }
        code += '}\n';
        return code;
    };

    window.RobotGenerators['robot_find_best_direction'] = function(block) {
        return ['(await findBestDirection())', 0];
    };

    window.RobotGenerators['robot_calibrate_turns'] = function(block) {
        const speed = block.getFieldValue('SPEED');
        return 'await calibrateTurning(' + speed + ');\n';
    };
}

window.generateRobotCode = function(workspace) {
    let code = '';
    const topBlocks = workspace.getTopBlocks(true);

    for (let block of topBlocks) {
        code += generateBlockCode(block);
    }

    return code;
};

// Rekursive Block-Code-Generierung
function generateBlockCode(block) {
    if (!block) return '';

    let code = '';
    const generator = window.RobotGenerators[block.type];

    if (generator) {
        const result = generator(block);
        code = Array.isArray(result) ? result[0] : result;
    } else {
        console.warn('Kein Generator für Block-Typ:', block.type);
    }

    // Für Statement-Blocks
    if (!block.outputConnection) {
        const nextBlock = block.getNextBlock();
        if (nextBlock) {
            code += generateBlockCode(nextBlock);
        }
    }

    return code;
}

// Initialisierung
function initRobotBlocks() {
    console.log('Initialisiere Roboter-Blöcke...');

    defineRobotBlocks();
    defineRobotGenerators();

    // Blockly.JavaScript Setup
    if (typeof Blockly !== 'undefined') {
        if (!Blockly.JavaScript) {
            Blockly.JavaScript = {};
        }

        // Registriere Generatoren
        for (let key in window.RobotGenerators) {
            Blockly.JavaScript[key] = window.RobotGenerators[key];
        }

        // Helper-Funktionen
        if (!Blockly.JavaScript.statementToCode) {
            Blockly.JavaScript.statementToCode = getStatementCode;
        }

        if (!Blockly.JavaScript.valueToCode) {
            Blockly.JavaScript.valueToCode = function(block, name) {
                return getValueCode(block, name)[0];
            };
        }

        // workspaceToCode
        if (!Blockly.JavaScript.workspaceToCode) {
            Blockly.JavaScript.workspaceToCode = window.generateRobotCode;
        }
    }

    window.robotBlocksInitialized = true;
    window.dispatchEvent(new CustomEvent('robotBlocksReady'));

    console.log('Roboter-Blöcke bereit!');
}

// Starte Initialisierung
if (typeof Blockly !== 'undefined') {
    initRobotBlocks();
} else {
    window.addEventListener('load', function() {
        if (typeof Blockly !== 'undefined') {
            initRobotBlocks();
        }
    });
}