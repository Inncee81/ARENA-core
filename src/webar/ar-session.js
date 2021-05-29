import {ARSource} from './ar-source.js';
import {Preprocessor} from './preprocessor.js';
import {Base64Binary} from './apriltag/base64_binary.js';
import * as Comlink from 'comlink';

const HIDDEN_CLASS = 'a-hidden';
const COMPUTER_VISION_DATA = 'cv_data';

const DTAG_ERROR_THRESH = 5e-6;
const MOVE_THRESH = 0.05;
const ROT_THRESH = 0.087;

window.processCV = async function(frame) {
    const cvDataEvent = new CustomEvent(
        COMPUTER_VISION_DATA,
        {detail: frame},
    );
    window.dispatchEvent(cvDataEvent);
};

AFRAME.registerComponent('arena-webar-session', {
    schema: {
        enabled: {type: 'boolean', default: true},
        drawTagsEnabled: {type: 'boolean', default: true},
        quadSigma: {type: 'number', default: 0.2},
        imgWidth: {type: 'number', default: 1280},
        imgHeight: {type: 'number', default: 720},
        cx: {type: 'number', default: 640},
        cy: {type: 'number', default: 360},
        fx: {type: 'number', default: 1280},
        fy: {type: 'number', default: 1280},
    },

    init: function() {
        this.isWebXRViewer = navigator.userAgent.includes('WebXRViewer');

        this.dtagMatrix = new THREE.Matrix4();
        this.rigMatrix = new THREE.Matrix4();

        this.vioMatrixPrev = new THREE.Matrix4();
        this.vioMatrix = new THREE.Matrix4();
        this.vioMatrixInv = new THREE.Matrix4();
        this.vioMatrixDiff = new THREE.Matrix4();
        this.vioPosDiff = new THREE.Vector3();

        this.tagPoseMatrix = new THREE.Matrix4();
        this.identityMatrix = new THREE.Matrix4();
        this.tagPoseRot = new THREE.Quaternion();
        // this.vioRot = new THREE.Quaternion();
        // this.vioPos = new THREE.Vector3();

        this.cvPerfTrack = Array(10).fill(16.66, 0, 10);

        this.FLIPMATRIX = new THREE.Matrix4();
        this.FLIPMATRIX.set(
            1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, -1, 0,
            0, 0, 0, 1,
        );

        this.originMatrix = new THREE.Matrix4();
        this.originMatrix.set( // row-major
            1, 0, 0, 0,
            0, 0, 1, 0,
            0, -1, 0, 0,
            0, 0, 0, 1,
        );

        this.ORIGINTAG = {
            id: 'ORIGIN',
            uuid: 'ORIGIN',
            pose: this.originMatrix,
        };

        this.aprilTags = {
            0: this.ORIGINTAG,
        };

        this.bufIndex = 0;
        this.cvRate = 0;
        this.cvThrottle = 0;

        this.updateAprilTags();

        if (this.isWebXRViewer) {
            this.onWebXRInit();
        } else {
            this.arSource = new ARSource();
            this.arSource.init()
                .then(this.onExperimentalWebARInit.bind(this))
                .catch((err) => {
                    console.warn(err);
                });
        }
    },

    updateAvgCVRate: function(lastInterval) {
        this.cvPerfTrack.shift();
        this.cvPerfTrack.push(lastInterval);
        const avg = this.cvPerfTrack.reduce((a, c) => a + c) / this.cvPerfTrack.length;
        this.cvRate = Math.ceil(60 / Math.max(1, Math.min(60, 1000 / avg)));
    },

    vioFilter: function(vioPrev, vioCur) {
        // posediff = pose2 @ np.linalg.inv(pose1)
        this.vioMatrixDiff.multiplyMatrices(vioPrev, vioCur);

        // np.linalg.norm(posediff[0:3, 3])
        const moveDiff = this.vioPosDiff.setFromMatrixPosition(
            this.vioMatrixDiff).length();
        if (moveDiff > MOVE_THRESH) {
            // console.log('Move Threshold Exceeded: ' + moveDiff);
            return false;
        }

        // math.acos((np.trace(posediff[0:3, 0:3]) - 1) / 2)
        const rotDiff = Math.acos(
            (this.vioMatrixDiff.elements[0] +
             this.vioMatrixDiff.elements[5] +
             this.vioMatrixDiff.elements[10] - 1) / 2);
        if (rotDiff > ROT_THRESH) {
            // console.log('Move Threshold Exceeded: ' + moveDiff);
            return false;
        }
        return true;
    },

    updateAprilTags: async function() {
        if (ARENA.clientCoords === undefined) {
            console.error('No device location! Cannot query ATLAS.');
            return false;
        }

        const position = ARENA.clientCoords;
        // limit to 3s update interval
        if (new Date() - ARENA.lastAprilTagUpdate < 3 * 1000) {
            return false;
        }

        const _this = this;
        fetch(ARENA.ATLASurl +
            '/lookup/geo?objectType=apriltag&distance=20&units=km&lat=' +
            position.latitude + '&long=' + position.longitude)
            .then((response) => {
                window.ARENA.lastAprilTagUpdate = new Date();
                return response.json();
            })
            .then((data) => {
                data.forEach((tag) => {
                    const tagid = tag.name.substring(9);
                    if (tagid !== 0) {
                        if (tag.pose && Array.isArray(tag.pose)) {
                            const tagMatrix = new THREE.Matrix4();
                            tagMatrix.fromArray(tag.pose.flat()); // comes in row-major, loads col-major
                            tagMatrix.transpose(); // flip properly to row-major
                            _this.aprilTags[tagid] = {
                                id: tagid,
                                uuid: tag.id,
                                pose: tagMatrix,
                            };
                        }
                    }
                });
            })
            .finally(() => {
                // Merge in apriltag system
            });
        return true;
    },

    getAprilTagWithId: function(id) {
        const tagSystem = document.querySelector('a-scene').systems['apriltag'];
        if (tagSystem !== undefined) {
            const sysTag = tagSystem.get(id);
            if (sysTag !== undefined) {
                return {
                    id: sysTag.data.tagid,
                    uuid: `apriltag_${sysTag.el.id}`,
                    pose: sysTag.el.object3D.matrixWorld,
                };
            }
        }
        return this.aprilTags[id];
    },

    onWebXRInit: async function() {
        const env = document.getElementById('env');
        env.setAttribute('visible', false);

        this.setupCursor();

        await this.apriltagInit();

        window.addEventListener(COMPUTER_VISION_DATA, this.webXRViewerProcessCVData.bind(this));
    },

    onExperimentalWebARInit: async function(source) {
        const data = this.data;
        const el = this.el;

        document.body.appendChild(source);

        // hide environment and make scene transparent
        const env = document.getElementById('env');
        env.setAttribute('visible', false);

        // hide ar/vr buttons
        this.hideVRButtons();

        // hide icons
        const icons = document.getElementById('icons-div');
        icons.style.display = 'none';
        const chatIcons = document.getElementsByClassName('chat-button-group')[0];
        chatIcons.style.display = 'none';

        const camera = document.getElementById('my-camera');
        // disable press and move controls
        camera.setAttribute('press-and-move', 'enabled', false);
        // remove dragging to rotate scene
        camera.setAttribute('look-controls', 'touchEnabled', false);
        // enable aframe's usage of gyro
        camera.setAttribute('look-controls', 'magicWindowTrackingEnabled', true);

        // create preprocessor
        this.preprocessor = new Preprocessor(data.imgWidth, data.imgHeight);
        this.preprocessor.setKernelSigma(data.quadSigma);
        this.preprocessor.attachElem(source);

        this.grayscaleImg = new Uint8Array(data.imgWidth * data.imgHeight);

        // create overlay canvas to draw detections, if needed
        if (data.drawTagsEnabled) {
            this.overlayCanvas = document.createElement('canvas');
            this.overlayCanvas.id = 'ar-overlay';
            this.overlayCanvas.width = data.imgWidth;
            this.overlayCanvas.height = data.imgHeight;
            this.overlayCanvas.style.position = 'absolute';
            this.overlayCanvas.style.top = '0px';
            this.overlayCanvas.style.left = '0px';
            this.overlayCanvas.style.zIndex = '-1';
            document.body.appendChild(this.overlayCanvas);
        }

        el.addState('ar-mode');
        el.resize();

        this.setupCursor();

        this.onResize();
        window.addEventListener('resize', this.onResize.bind(this));

        // download Apriltag WASM module and start processing loop
        await this.apriltagInit();
    },

    apriltagInit: async function() {
        const Apriltag = Comlink.wrap(new Worker('./apriltag/apriltag.js'));
        this.aprilTag = await new Apriltag(Comlink.proxy(this.onApriltagInit.bind(this)));
    },

    onApriltagInit: function() {
        const data = this.data;
        const el = this.el;

        this.fx = data.fx; this.fy = data.fy; this.cx = data.cx; this.cy = data.cy;
        this.aprilTag.set_camera_info(this.fx, this.fy, this.cx, this.cy);

        if (!this.isWebXRViewer) {
            this.experimentalWebARProcessCVData();
        }
    },

    setupCursor: function() {
        const urlParams = new URLSearchParams(window.location.search);

        // create cursor
        let cursor = document.getElementById('mouse-cursor');
        const cursorParent = cursor.parentNode;
        cursorParent.removeChild(cursor);
        cursor = document.createElement('a-cursor');
        cursor.setAttribute('fuse', false);
        cursor.setAttribute('scale', '0.1 0.1 0.1');
        cursor.setAttribute('position', '0 0 -0.1');
        if (urlParams.get('noreticle')) {
            cursor.setAttribute('material', 'transparent: "true"; opacity: 0');
        } else {
            cursor.setAttribute('color', '#555');
        }
        cursor.setAttribute('max-distance', '10000');
        cursorParent.appendChild(cursor);

        window.lastMouseTarget = undefined;

        // handle tap events
        document.addEventListener('mousedown', function(e) {
            if (window.lastMouseTarget) {
                const el = document.getElementById(window.lastMouseTarget);
                const elPos = new THREE.Vector3();
                el.object3D.getWorldPosition(elPos);

                const intersection = {
                    x: elPos.x,
                    y: elPos.y,
                    z: elPos.z,
                };
                el.emit('mousedown', {
                    'clicker': window.ARENA.camName,
                    'intersection': {
                        point: intersection,
                    },
                    'cursorEl': true,
                }, false);
            }
        });

        document.addEventListener('mouseup', function(e) {
            if (window.lastMouseTarget) {
                const el = document.getElementById(window.lastMouseTarget);
                const elPos = new THREE.Vector3();
                el.object3D.getWorldPosition(elPos);
                const intersection = {
                    x: elPos.x,
                    y: elPos.y,
                    z: elPos.z,
                };
                el.emit('mouseup', {
                    'clicker': window.ARENA.camName,
                    'intersection': {
                        point: intersection,
                    },
                    'cursorEl': true,
                }, false);
            }
        });
    },

    hideVRButtons: function() {
        const data = this.data;
        const el = this.el;

        const enterAREl = el.components['vr-mode-ui'].enterAREl;
        enterAREl.classList.add(HIDDEN_CLASS);
        const enterVREl = el.components['vr-mode-ui'].enterVREl;
        enterVREl.classList.add(HIDDEN_CLASS);
    },

    onResize: function() {
        const data = this.data;
        const el = this.el;

        // resize AR pass through
        this.arSource.resize(window.innerWidth, window.innerHeight);
        if (data.drawTagsEnabled) {
            this.arSource.copyElementSizeTo(this.overlayCanvas);
        }

        // set new camera projection matrix parameters
        if (window.innerWidth > data.imgWidth || window.innerHeight > data.imgHeight) {
            el.camera.fov = 31; // found empirically
        } else {
            el.camera.fov = 26; // found empirically
        }
        el.camera.aspect = window.innerWidth / window.innerHeight;
        el.camera.near = 0.001; // webxr viewer parameters
        el.camera.far = 1000.0;
        el.camera.updateProjectionMatrix();

        // webxr:
        // el.camera.projectionMatrix.elements[0] = 1.7113397121429443; // 1.6807010173797607
        // el.camera.projectionMatrix.elements[1] = 0;
        // el.camera.projectionMatrix.elements[2] = 0;
        // el.camera.projectionMatrix.elements[3] = 0;
        // el.camera.projectionMatrix.elements[4] = 0;
        // el.camera.projectionMatrix.elements[5] = 3.5782558917999268; // 2.9894068241119385
        // el.camera.projectionMatrix.elements[6] = 0;
        // el.camera.projectionMatrix.elements[7] = 0;
        // el.camera.projectionMatrix.elements[8] = 0;
        // el.camera.projectionMatrix.elements[9] = 0;
        // el.camera.projectionMatrix.elements[10] = -1.0000009536743164;
        // el.camera.projectionMatrix.elements[11] = -1;
        // el.camera.projectionMatrix.elements[12] = 0;
        // el.camera.projectionMatrix.elements[13] = 0;
        // el.camera.projectionMatrix.elements[14] = -0.001000000978820026;
        // el.camera.projectionMatrix.elements[15] = 0;
    },

    experimentalWebARProcessCVData: async function() {
        const data = this.data;
        const el = this.el;

        // this.cvThrottle++;
        // if (this.cvThrottle % this.cvRate) {
        //     requestAnimationFrame(this.experimentalWebARProcessCVData.bind(this));
        // }
        // const start = Date.now();

        const imageData = this.preprocessor.getPixels();
        // grab only one channel; already grayscaled!
        for (let i = 0, j = 0; i < data.imgWidth * data.imgHeight * 4; i+=4, j++) {
            this.grayscaleImg[j] = imageData[i];
        }

        // detect apriltags
        const detections = await this.processCVData(this.grayscaleImg, data.imgWidth, data.imgHeight);
        if (data.drawTagsEnabled) {
            this.drawTags(detections);
        }

        // this.updateAvgCVRate(Date.now() - start);

        requestAnimationFrame(this.experimentalWebARProcessCVData.bind(this));
    },

    webXRViewerProcessCVData: async function(e) {
        const data = this.data;
        const el = this.el;

        this.cvThrottle++;
        if (this.cvThrottle % this.cvRate) {
            return;
        }
        const start = Date.now();

        const frame = e.detail;
        // set camera intrinsics for pose detection
        if (frame._camera.cameraIntrinsics[0] != this.fx || frame._camera.cameraIntrinsics[4] != this.fy ||
            frame._camera.cameraIntrinsics[6] != this.cx || frame._camera.cameraIntrinsics[7] != this.cy) {
            this.fx = frame._camera.cameraIntrinsics[0];
            this.fy = frame._camera.cameraIntrinsics[4];
            this.cx = frame._camera.cameraIntrinsics[6];
            this.cy = frame._camera.cameraIntrinsics[7];
            this.aprilTag.set_camera_info(this.fx, this.fy, this.cx, this.cy);
        }

        const imgWidth = frame._buffers[this.bufIndex].size.width;
        const imgHeight = frame._buffers[this.bufIndex].size.height;

        const byteArray = Base64Binary.decodeArrayBuffer(frame._buffers[this.bufIndex]._buffer);
        // cut u and v values; grayscale image is just the y values
        const grayscaleImg = new Uint8Array(byteArray.slice(0, imgWidth * imgHeight));

        await this.processCVData(grayscaleImg, imgWidth, imgHeight);

        this.updateAvgCVRate(Date.now() - start);
    },

    processCVData: async function(grayscaleImg, imgWidth, imgHeight) {
        this.vioMatrixPrev.copy(this.vioMatrix);

        const camera = document.getElementById('my-camera');
        const camParentMat = camera.object3D.parent.matrixWorld;
        const camMat = camera.object3D.matrixWorld;

        this.vioMatrix.copy(camParentMat).invert(); // vioMatrix.getInverse(camParent);
        this.vioMatrix.multiply(camMat);
        this.vioMatrixInv.copy(this.vioMatrix).invert(); // vioMatrixT.getInverse(vioMatrix);
        if (!this.vioFilter(this.vioMatrixPrev, this.vioMatrixInv)) {
            return;
        }

        // this.vioRot.setFromRotationMatrix(this.vioMatrix);
        // this.vioPos.setFromMatrixPosition(this.vioMatrix);

        // detect apriltags
        const detections = await this.aprilTag.detect(grayscaleImg, imgWidth, imgHeight);
        if (detections.length > 0) {
            for (const d of detections) {
                // console.log(d.pose.e);
                if (this.isWebXRViewer && d.pose.e > DTAG_ERROR_THRESH) {
                    continue;
                }

                const timestamp = new Date();
                const jsonMsg = {scene: ARENA.renderParam, timestamp: timestamp, camera_id: ARENA.camName};

                // search for tag with detection id
                // known tag from ATLAS (includes Origin tag)
                const indexedTag = this.getAprilTagWithId(d.id);

                // if tag has pose, update camera position based on pose
                if (indexedTag?.pose) {
                    const refTag = indexedTag;

                    // calculate pose
                    const rigPose = this.getRigPoseFromAprilTag(d.pose, refTag.pose);

                    // update camera
                    document.getElementById('cameraSpinner').object3D.quaternion.setFromRotationMatrix(rigPose);
                    document.getElementById('cameraRig').object3D.position.setFromMatrixPosition(rigPose);
                } else {
                    if (this.rigMatrix.equals(this.identityMatrix)) {
                        console.log('Client apriltag solver no calculated rigMatrix yet, zero on origin tag first');
                    } else {
                        const tagPose = this.getTagPoseFromRig(d.pose);
                        this.tagPoseRot.setFromRotationMatrix(tagPose);

                        Object.assign(jsonMsg, {
                            object_id: 'apriltag_' + d.id,
                            action: 'update',
                            type: 'object',
                            data: {
                                'position': {
                                    'x': tagPose.elements[12],
                                    'y': tagPose.elements[13],
                                    'z': tagPose.elements[14],
                                },
                                'rotation': {
                                    'x': this.tagPoseRot.x,
                                    'y': this.tagPoseRot.y,
                                    'z': this.tagPoseRot.z,
                                    'w': this.tagPoseRot.w,
                                },
                            },
                        });
                        ARENA.Mqtt.publish(
                            'realm/s/' + ARENA.renderParam + '/apriltag_' +
                            d.id, JSON.stringify(jsonMsg));
                    }
                }
            }
            const ids = detections.map((tag) => tag.id);
            console.log('April Tag IDs Detected: ' + ids.join(', '));
        }

        return detections;
    },

    getRigPoseFromAprilTag: function(dtagPose, refTagPose) {
        const r = dtagPose.R;
        const t = dtagPose.t;

        this.dtagMatrix.set( // Transposed rotation
            r[0][0], r[1][0], r[2][0], t[0],
            r[0][1], r[1][1], r[2][1], t[1],
            r[0][2], r[1][2], r[2][2], t[2],
            0, 0, 0, 1,
        );
        this.dtagMatrix.premultiply(this.FLIPMATRIX);
        this.dtagMatrix.multiply(this.FLIPMATRIX);

        // Python rig_pose = ref_tag_pose @ np.linalg.inv(dtag_pose) @ np.linalg.inv(vio_pose)
        this.dtagMatrix.copy(this.dtagMatrix).invert();
        this.rigMatrix.identity();
        this.rigMatrix.multiplyMatrices(refTagPose, this.dtagMatrix);
        this.rigMatrix.multiply(this.vioMatrixInv);

        return this.rigMatrix;
    },

    getTagPoseFromRig: function(dtag) {
        const r = dtag.R;
        const t = dtag.t;

        this.dtagMatrix.set( // Transposed rotation
            r[0][0], r[1][0], r[2][0], t[0],
            r[0][1], r[1][1], r[2][1], t[1],
            r[0][2], r[1][2], r[2][2], t[2],
            0, 0, 0, 1,
        );
        this.dtagMatrix.premultiply(this.FLIPMATRIX);
        this.dtagMatrix.multiply(this.FLIPMATRIX);

        // Python ref_tag_pose = rig_pose @ vio_pose @ dtag_pose
        this.tagPoseMatrix.copy(this.rigMatrix);
        this.tagPoseMatrix.multiply(this.vioMatrix);
        this.tagPoseMatrix.multiply(this.dtagMatrix);

        return this.tagPoseMatrix;
    },

    drawTags: function(tags) {
        const data = this.data;

        if (!tags || !data.drawTagsEnabled) {
            return;
        }

        const overlayCtx = this.overlayCanvas.getContext('2d');
        overlayCtx.clearRect(0, 0, this.overlayCanvas.width, this.overlayCanvas.height);

        for (const tag of tags) {
            this.drawTag(tag);
        }
    },

    drawTag: function(tag) {
        const data = this.data;

        if (!data.drawTagsEnabled) {
            return;
        }

        const overlayCtx = this.overlayCanvas.getContext('2d');

        overlayCtx.beginPath();
        overlayCtx.lineWidth = 3;
        overlayCtx.strokeStyle = 'blue';
        overlayCtx.moveTo(tag.corners[0].x, tag.corners[0].y);
        overlayCtx.lineTo(tag.corners[1].x, tag.corners[1].y);
        overlayCtx.lineTo(tag.corners[2].x, tag.corners[2].y);
        overlayCtx.lineTo(tag.corners[3].x, tag.corners[3].y);
        overlayCtx.lineTo(tag.corners[0].x, tag.corners[0].y);
        overlayCtx.stroke();

        overlayCtx.font = 'bold 20px Arial';
        overlayCtx.textAlign = 'center';
        overlayCtx.fillStyle = 'red';
        overlayCtx.fillText(tag.id, tag.center.x, tag.center.y);
    },

    update(oldData) {
        const data = this.data;
        if (data.enabled) {

        }
    },

    tick: function(t, dt) {

    },
});
