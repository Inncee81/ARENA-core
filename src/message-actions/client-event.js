/* global THREE */

/**
 * @fileoverview Handle client event messages
 *
 * Open source software under the terms in /LICENSE
 * Copyright (c) 2020, The CONIX Research Center. All rights reserved.
 * @date 2020
 */

import {Parser} from './parser.js';

/**
 * Client Event handler
 */
export class ClientEvent {
    /**
     * Client Event handler
     * @param {object} message message to be parsed
     */
    static handle(message) {
        const id = message.id;
        const data = message.data;

        // ignore goto-url and textinput events
        if (message.type == 'goto-url' || message.type == 'textinput') {
            return;
        }

        const entityEl = document.getElementById(id);
        if (!entityEl) {
            console.error('clientEvent:', `Object with object_id "${id}" does not exist!`);
            return;
        }

        let point = null;
        if (data.position) {
            point = new THREE.Vector3(
                parseFloat(data.position.x),
                parseFloat(data.position.y),
                parseFloat(data.position.z),
            );
        } else {
            console.warning('clientEvent:', 'Malformed message (no data.position):', JSON.stringify(message));
        }

        const clicker = data.source;
        switch (message.type) {
        case 'collision':
            // emit a synthetic click event with ugly data syntax
            entityEl.emit('mousedown', {
                'clicker': clicker,
                'intersection': {
                    point: point,
                },
            }, false);
            break;
        case 'mousedown':
            // emit a synthetic click event with ugly data syntax
            entityEl.emit('mousedown', {
                'clicker': clicker,
                'intersection': {
                    point: point,
                },
            }, false);
            break;
        case 'mouseup':
            // emit a synthetic click event with ugly data syntax
            entityEl.emit('mouseup', {
                'clicker': clicker,
                'intersection': {
                    point: point,
                },
            }, false);
            break;
        default: // handle others here like mouseenter / mouseleave
            break;
        }
    }
}
