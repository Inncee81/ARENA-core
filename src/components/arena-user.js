/* global AFRAME, ARENA, THREE */

/**
 * @fileoverview Another user's camera in the ARENA. Handles Jitsi and display name updates.
 *
 * Open source software under the terms in /LICENSE
 * Copyright (c) 2020, The CONIX Research Center. All rights reserved.
 * @date 2020
 */

/**
 * Workaround for AEC when using Web Audio API (https://bugs.chromium.org/p/chromium/issues/detail?id=687574)
 * https://github.com/mozilla/hubs/blob/master/src/systems/audio-system.js
 * @param {Object} gainNode
 * @private
 */
async function enableChromeAEC(gainNode) {
    /**
     *  workaround for: https://bugs.chromium.org/p/chromium/issues/detail?id=687574
     *  1. grab the GainNode from the scene's THREE.AudioListener
     *  2. disconnect the GainNode from the AudioDestinationNode (basically the audio out),
     *     this prevents hearing the audio twice.
     *  3. create a local webrtc connection between two RTCPeerConnections (see this example: https://webrtc.github.io/samples/src/content/peerconnection/pc1/)
     *  4. create a new MediaStreamDestination from the scene's THREE.AudioContext and connect the GainNode to it.
     *  5. add the MediaStreamDestination's track  to one of those RTCPeerConnections
     *  6. connect the other RTCPeerConnection's stream to a new audio element.
     *  All audio is now routed through Chrome's audio mixer, thus enabling AEC,
     *  while preserving all the audio processing that was performed via the WebAudio API.
     */

    const audioEl = new Audio();
    audioEl.setAttribute('autoplay', 'autoplay');
    audioEl.setAttribute('playsinline', 'playsinline');

    const context = THREE.AudioContext.getContext();
    const loopbackDestination = context.createMediaStreamDestination();
    const outboundPeerConnection = new RTCPeerConnection();
    const inboundPeerConnection = new RTCPeerConnection();

    const onError = (e) => {
        console.error('RTCPeerConnection loopback initialization error', e);
    };

    outboundPeerConnection.addEventListener('icecandidate', (e) => {
        inboundPeerConnection.addIceCandidate(e.candidate).catch(onError);
    });

    inboundPeerConnection.addEventListener('icecandidate', (e) => {
        outboundPeerConnection.addIceCandidate(e.candidate).catch(onError);
    });

    inboundPeerConnection.addEventListener('track', (e) => {
        audioEl.srcObject = e.streams[0];
    });

    try {
        /* The following should never fail, but just in case, we won't disconnect/reconnect
           the gainNode unless all of this succeeds */
        loopbackDestination.stream.getTracks().forEach((track) => {
            outboundPeerConnection.addTrack(track, loopbackDestination.stream);
        });

        const offer = await outboundPeerConnection.createOffer();
        outboundPeerConnection.setLocalDescription(offer);
        await inboundPeerConnection.setRemoteDescription(offer);

        const answer = await inboundPeerConnection.createAnswer();
        inboundPeerConnection.setLocalDescription(answer);
        outboundPeerConnection.setRemoteDescription(answer);

        gainNode.disconnect();
        if (ARENA.Jitsi.spatialAudioOn) {
            gainNode.connect(context.destination);
        } else {
            gainNode.connect(loopbackDestination);
        }
    } catch (e) {
        onError(e);
    }
}

/**
 * Another user's camera in the ARENA. Handles Jitsi and display name updates.
 * @module arena-user
 * @property {color} [color=white] - The color for the user's name text.
 * @property {string} [headModelPath=/store/models/robobit.glb] - Path to user head model
 * @property {string} [jitsiId] - User jitsi id.
 * @property {string} [displayName] - User display name.
 * @property {boolean} [hasAudio=false] - Whether the user has audio on.
 * @property {boolean} [hasVideo=false] - Whether the user has video on.
 *
 */
AFRAME.registerComponent('arena-user', {
    schema: {
        color: {type: 'color', default: 'white'},
        headModelPath: {type: 'string', default: '/store/models/robobit.glb'},
        jitsiId: {type: 'string', default: ''},
        displayName: {type: 'string', default: ''},
        hasAudio: {type: 'boolean', default: false},
        hasVideo: {type: 'boolean', default: false},
    },

    init: function() {
        const data = this.data;
        const el = this.el;
        const name = el.id;

        el.setAttribute('rotation.order', 'YXZ');
        el.object3D.position.set(0, ARENA.defaults.camHeight, 0);
        el.object3D.rotation.set(0, 0, 0);

        const decodeName = decodeURI(name.split('_')[2]);
        const personName = decodeName.normalize('NFD').replace(/[\u0300-\u036f]/g, '');
        this.headText = document.createElement('a-text');
        this.headText.setAttribute('id', 'headtext_' + name);
        this.headText.setAttribute('value', personName);
        this.headText.setAttribute('position', '0 0.45 0.05');
        this.headText.setAttribute('side', 'double');
        this.headText.setAttribute('align', 'center');
        this.headText.setAttribute('anchor', 'center');
        this.headText.setAttribute('scale', '0.4 0.4 0.4');
        this.headText.setAttribute('rotation', '0 180 0');
        this.headText.setAttribute('color', data.color);
        this.headText.setAttribute('width', 5); // try setting last

        this.headModel = document.createElement('a-entity');
        this.headModel.setAttribute('id', 'head-model_' + name);
        this.headModel.setAttribute('rotation', '0 180 0');
        this.headModel.object3D.scale.set(1, 1, 1);
        this.headModel.setAttribute('dynamic-body', 'type', 'static');
        this.headModel.setAttribute('gltf-model', `url(${data.headModelPath})`);
        this.headModel.setAttribute('attribution', 'extractAssetExtras', true);

        el.appendChild(this.headText);
        el.appendChild(this.headModel);
        this.drawMicrophone();

        this.videoID = null;
        this.audioID = null;
        this.distReached = null;

        this.frustum = new THREE.Frustum();

        this.tick = AFRAME.utils.throttleTick(this.tick, 1000, this);
    },

    aec(listener) {
        // sorta fixes chrome echo bug
        const audioCtx = THREE.AudioContext.getContext();
        const resume = () => {
            audioCtx.resume();
            setTimeout(function() {
                if (audioCtx.state === 'running') {
                    if (!AFRAME.utils.device.isMobile() && /chrome/i.test(navigator.userAgent)) {
                        enableChromeAEC(listener.gain);
                    }
                    document.body.removeEventListener('touchmove', resume, false);
                    document.body.removeEventListener('mousemove', resume, false);
                }
            }, 0);
        };
        document.body.addEventListener('touchmove', resume, false);
        document.body.addEventListener('mousemove', resume, false);
    },

    drawMicrophone() {
        const el = this.el;

        const name = 'muted_' + el.id;
        let micIconEl = document.querySelector('#' + name);
        if (!micIconEl) {
            micIconEl = document.createElement('a-image');
            micIconEl.setAttribute('id', name);
            micIconEl.setAttribute('scale', '0.2 0.2 0.2');
            micIconEl.setAttribute('position', '0 0.3 0.045');
            micIconEl.setAttribute('src', 'url(/src/icons/images/audio-off.png)');
            el.appendChild(micIconEl);
        }
    },

    removeMicrophone() {
        const el = this.el;

        const name = 'muted_' + el.id;
        const micIconEl = document.querySelector('#' + name);
        if (micIconEl) {
            el.removeChild(micIconEl);
        }
    },

    drawVideoCube() {
        const el = this.el;

        // attach video to head
        const videoCube = document.createElement('a-box');
        videoCube.setAttribute('id', this.videoID + 'cube');
        videoCube.setAttribute('position', '0 0 0');
        videoCube.setAttribute('scale', '0.6 0.4 0.6');
        videoCube.setAttribute('material', 'shader', 'flat');
        videoCube.setAttribute('src', `#${this.videoID}`); // video only! (no audio)
        videoCube.setAttribute('material-extras', 'encoding', 'sRGBEncoding');
        videoCube.setAttribute('material-extras', 'needsUpdate', 'true');

        const videoCubeDark = document.createElement('a-box');
        videoCubeDark.setAttribute('id', this.videoID + 'cubeDark');
        videoCubeDark.setAttribute('position', '0 0 0.01');
        videoCubeDark.setAttribute('scale', '0.61 0.41 0.6');
        videoCubeDark.setAttribute('material', 'shader', 'flat');
        videoCubeDark.setAttribute('transparent', 'true');
        videoCubeDark.setAttribute('color', 'black');
        videoCubeDark.setAttribute('opacity', '0.8');

        el.appendChild(videoCube);
        el.appendChild(videoCubeDark);
    },

    removeVideoCube() {
        const el = this.el;

        // remove video cubes
        const vidCube = document.getElementById(this.videoID + 'cube');
        if (el.contains(vidCube)) {
            el.removeChild(vidCube);
        }

        const vidCubeDark = document.getElementById(this.videoID + 'cubeDark');
        if (el.contains(vidCubeDark)) {
            el.removeChild(vidCubeDark);
        }
    },

    updateVideo() {
        const data = this.data;
        if (!data) return;

        /* Handle Jitsi Video */
        this.videoID = `video${data.jitsiId}`;
        if (data.hasVideo) {
            if (!ARENA.Jitsi.getVideoTrack(data.jitsiId)) {
                return;
            }

            const jistiVideo = document.getElementById(this.videoID);
            if (jistiVideo) {
                const vidCube = document.getElementById(this.videoID + 'cube');
                if (!vidCube) {
                    this.drawVideoCube();
                }
            }
        } else {
            this.removeVideoCube();
        }
    },

    createAudio() {
        const el = this.el;
        el.setAttribute('sound', `src: #${this.audioID}`);

        this.aec(el.sceneEl.audioListener);

        el.setAttribute('sound', 'positional: true');
        if (ARENA.refDistance) {
            el.setAttribute('sound', `refDistance: ${ARENA.refDistance}`);
        }
        if (ARENA.rolloffFactor) {
            el.setAttribute('sound', `rolloffFactor: ${ARENA.rolloffFactor}`);
        }
        if (ARENA.distanceModel) {
            el.setAttribute('sound', `distanceModel: ${ARENA.distanceModel}`);
        }
        if (ARENA.volume) {
            el.setAttribute('sound', `volume: ${ARENA.volume}`);
        }
    },

    updateAudio() {
        const data = this.data;
        if (!data) return;
        const el = this.el;

        /* Handle Jitsi Audio */
        this.audioID = `audio${data.jitsiId}`;
        if (data.hasAudio) {
            // set up positional audio, but only once per camera
            if (!ARENA.Jitsi.getAudioTrack(data.jitsiId)) {
                return;
            }

            const jitsiAudio = document.getElementById(this.audioID);
            if (jitsiAudio) {
                const sound = el.components.sound;
                if (!this.distReached && !sound) {
                    this.createAudio();
                }
                this.removeMicrophone();
            }
        } else {
            this.drawMicrophone();
        }
    },

    muteAudio() {
        const el = this.el;
        const jistiAudio = document.getElementById(this.audioID);
        if (jistiAudio) {
            jistiAudio.srcObject.getTracks().forEach((t) => t.enabled = false);
        }
        el.removeAttribute('sound');
    },

    unmuteAudio() {
        const jistiAudio = document.getElementById(this.audioID);
        if (jistiAudio) {
            jistiAudio.srcObject.getTracks().forEach((t) => t.enabled = true);
        }
    },

    muteVideo() {
        const jistiVideo = document.getElementById(this.videoID);
        if (jistiVideo) {
            if (!jistiVideo.paused) jistiVideo.pause();
            jistiVideo.srcObject.getTracks().forEach((t) => t.enabled = false);
        }
    },

    unmuteVideo() {
        const jistiVideo = document.getElementById(this.videoID);
        if (jistiVideo) {
            if (jistiVideo.paused) jistiVideo.play();
            jistiVideo.srcObject.getTracks().forEach((t) => t.enabled = true);
        }
    },

    update: function(oldData) {
        const data = this.data;

        if (data.displayName !== oldData.displayName) {
            const name = data.displayName.normalize('NFD').replace(/[\u0300-\u036f]/g, '');
            this.headText.setAttribute('value', name);
        }

        if (data.color !== oldData.color) {
            this.headText.setAttribute('color', data.color);
        }
    },

    tick: function() {
        const data = this.data;
        const el = this.el;

        // do periodic a/v updates
        if (ARENA.Jitsi && ARENA.Jitsi.ready && data.jitsiId) {
            this.updateVideo();
            this.updateAudio();
        }

        const camPos = document.getElementById('my-camera').object3D.position;
        const entityPos = el.object3D.position;
        const distance = camPos.distanceTo(entityPos);

        if (this.videoID) {
            // frustum culling for WebRTC video streams
            const vidCube = document.getElementById(this.videoID + 'cube');
            let inFieldOfView = true;
            if (el.contains(vidCube)) {
                const cam = document.getElementById('my-camera').sceneEl.camera;
                this.frustum.setFromProjectionMatrix(
                    new THREE.Matrix4().multiplyMatrices(
                        cam.projectionMatrix, cam.matrixWorldInverse));
                const bbox = new THREE.Box3().setFromObject(vidCube.object3D);
                inFieldOfView = this.frustum.intersectsBox(bbox);
            }

            // check if A/V cut off distance has been reached
            if (!inFieldOfView || distance > ARENA.maxAVDist) {
                this.muteVideo();
            } else {
                this.unmuteVideo();
            }
        }

        if (this.audioID) {
            // check if A/V cut off distance has been reached
            if (distance > ARENA.maxAVDist) {
                this.muteAudio();
                this.distReached = true;
            } else {
                this.unmuteAudio();
                this.distReached = false;
            }
        }
    },
});
