import {Logger} from './logger.js';

/**
 * Delete object handler
 */
export class Delete {
    /**
     * Delete handler
     * @param {object} message message to be parsed
     */
    static handle(message) {
        const id = message.id;
        if (id === undefined) {
            console.error('delete', 'Malformed message (no object_id):', JSON.stringify(message));
        }

        const entityEl = document.getElementById(id);
        if (!entityEl) {
            console.error('delete', `Object with object_id "${id}" does not exist!`);
            return;
        }

        const parentEl = entityEl.parentEl;
        if (parentEl) {
            parentEl.removeChild(entityEl);
        }
    }
}
