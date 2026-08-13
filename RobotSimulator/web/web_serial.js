
mergeInto(LibraryManager.library, {
  $WebSerial: {
    granted: [],      // ports the user has authorised; the index is the port name in C++
    port: null,
    reader: null,
    writer: null,
    rx: [],           // received bytes not yet handed to C++
    txQueue: [],      // writes issued while the open was still in flight
    isOpen: false,
    opening: false,
    requestError: '',

    refreshGranted: function () {
      if (!navigator.serial) return;
      navigator.serial.getPorts().then(function (ports) {
        WebSerial.granted = ports;
      }).catch(function () {});
    },

    pump: function () {
      if (!WebSerial.reader) return;
      WebSerial.reader.read().then(function (result) {
        if (result.value) {
          for (var i = 0; i < result.value.length; ++i) WebSerial.rx.push(result.value[i]);
        }
        if (result.done) return;
        WebSerial.pump();
      }).catch(function () {
        // An unplugged or closed device lands here; isOpen() then reports false.
        WebSerial.isOpen = false;
      });
    },

    reset: function () {
      WebSerial.reader = null;
      WebSerial.writer = null;
      WebSerial.port = null;
      WebSerial.isOpen = false;
      WebSerial.opening = false;
      WebSerial.rx = [];
      WebSerial.txQueue = [];
    }
  },

  webSerialInit__deps: ['$WebSerial'],
  webSerialInit: function () {
    WebSerial.refreshGranted();
  },

  webSerialRequestPort__deps: ['$WebSerial'],
  webSerialRequestPort: function () {
    WebSerial.requestError = '';
    if (!navigator.serial) {
      WebSerial.requestError =
          'This browser does not implement Web Serial. Chrome or Edge is required.';
      return;
    }
    navigator.serial.requestPort().then(function () {
      WebSerial.refreshGranted();
    }).catch(function (error) {
      // NotFoundError is the user dismissing the chooser, which is not worth reporting.
      if (error && error.name !== 'NotFoundError') {
        WebSerial.requestError = String(error.message || error);
      }
    });
  },

  webSerialGrantedCount__deps: ['$WebSerial'],
  webSerialGrantedCount: function () {
    WebSerial.refreshGranted();
    return WebSerial.granted.length;
  },

  webSerialDescribe__deps: ['$WebSerial', '$stringToNewUTF8'],
  webSerialDescribe: function (index) {
    var text = 'Serial device';
    try {
      var info = WebSerial.granted[index].getInfo();
      if (info && info.usbVendorId !== undefined) {
        var product = info.usbProductId !== undefined ? info.usbProductId.toString(16) : '????';
        text = 'USB ' + info.usbVendorId.toString(16) + ':' + product;
      }
    } catch (e) { /* getInfo is not available everywhere */ }
    return stringToNewUTF8(text);
  },

  webSerialLastError__deps: ['$WebSerial', '$stringToNewUTF8'],
  webSerialLastError: function () {
    return stringToNewUTF8(WebSerial.requestError || '');
  },

  webSerialOpen__deps: ['$WebSerial'],
  webSerialOpen: function (index, baudRate) {
    var port = WebSerial.granted[index];
    if (!port) return 0;
    WebSerial.reset();
    WebSerial.opening = true;
    WebSerial.port = port;
    port.open({ baudRate: baudRate }).then(function () {
      WebSerial.reader = port.readable.getReader();
      WebSerial.writer = port.writable.getWriter();
      WebSerial.isOpen = true;
      WebSerial.opening = false;
      // Anything written while the open was in flight goes out now, in order.
      var queued = WebSerial.txQueue;
      WebSerial.txQueue = [];
      for (var i = 0; i < queued.length; ++i) WebSerial.writer.write(queued[i]);
      WebSerial.pump();
    }).catch(function (error) {
      WebSerial.opening = false;
      WebSerial.isOpen = false;
      WebSerial.requestError = String(error.message || error);
    });
    return 1;
  },

  webSerialClose__deps: ['$WebSerial'],
  webSerialClose: function () {
    try { if (WebSerial.reader) WebSerial.reader.cancel(); } catch (e) {}
    try { if (WebSerial.writer) WebSerial.writer.close(); } catch (e) {}
    try { if (WebSerial.port) WebSerial.port.close(); } catch (e) {}
    WebSerial.reset();
  },

  webSerialIsOpen__deps: ['$WebSerial'],
  webSerialIsOpen: function () {
    // Counts an in-flight open as open so a caller writing immediately afterwards is not told
    // the link is dead; those writes are queued above.
    return (WebSerial.isOpen || WebSerial.opening) ? 1 : 0;
  },

  webSerialPending__deps: ['$WebSerial'],
  webSerialPending: function () {
    return WebSerial.rx.length;
  },

  webSerialTake__deps: ['$WebSerial'],
  webSerialTake: function (destination, count) {
    var taken = WebSerial.rx.splice(0, count);
    for (var i = 0; i < taken.length; ++i) HEAPU8[destination + i] = taken[i];
    return taken.length;
  },

  webSerialWrite__deps: ['$WebSerial'],
  webSerialWrite: function (textPointer) {
    var bytes = new TextEncoder().encode(UTF8ToString(textPointer));
    if (WebSerial.writer && WebSerial.isOpen) WebSerial.writer.write(bytes);
    else WebSerial.txQueue.push(bytes);
    return 1;
  }
});
