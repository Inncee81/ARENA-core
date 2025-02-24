/**
 * @fileoverview Interact with user account service
 *
 * Open source software under the terms in /LICENSE
 * Copyright (c) 2020, The CONIX Research Center. All rights reserved.
 * @date 2020
 */

/**
 * Wrapper class to perform requests to arena account
 */
export class ARENAUserAccount {
    /**
     * Internal call to perform xhr request
     */
    static _makeRequest(method, url, params = undefined) {
        return new Promise(function(resolve, reject) {
            let xhr = new XMLHttpRequest();
            xhr.open(method, url);
            const csrftoken = getCookie('csrftoken');
            xhr.setRequestHeader('X-CSRFToken', csrftoken);
            xhr.responseType = 'json';
            xhr.onload = function() {
                if (this.status >= 200 && this.status < 300) {
                    resolve(xhr.response);
                } else {
                    reject({
                        status: this.status,
                        statusText: xhr.statusText
                    });
                }
            };
            xhr.onerror = function() {
                reject({
                    status: this.status,
                    statusText: xhr.statusText,
                });
            };
            xhr.send(params);
        });
    }

    /**
     * @typedef {Object} UserAccountData
     * @param authenticated {boolean} is the user authenticated
     * @param type {string} auth type
     * @param username {string} user's name
     * @param fullname {string} user's full name
     * @param email {string} user's email
     */

    /**
     * Request user state data for client-side state management
     * @return {UserAccountData} object with user account data
     */
    static async userAuthState() {
        let result = await ARENAUserAccount._makeRequest('GET', `/user/user_state`);
        return result;
    }

    /**
     * Request scene names which the user has permission to from user database
     * @return {string[]]} list of scene names
     */
    static async userScenes() {
        let result = await ARENAUserAccount._makeRequest('GET', '/user/my_scenes');
        return result;
    }

    /**
     * Request a scene is added to the user database.
     * @param {string} scene_namespace name of the scene without namespace
     * @param {boolean} isPublic true when 'public' namespace is used, false for user namespace
     */
    static async requestUserNewScene(scene_namespace) {
        var params = new FormData();
        let result = await ARENAUserAccount._makeRequest('POST', `/user/scenes/${scene_namespace}`);
        return result;
    }

    /**
     * Request to delete scene permissions from user db
     * @param {string} scene_namespace name of the scene without namespace
     */
    static async requestDeleteUserScene(scene_namespace) {
        let result = await ARENAUserAccount._makeRequest('DELETE', `/user/scenes/${scene_namespace}`);
        return result;
    }
}
