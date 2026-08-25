const CONFIG = {
  PYTHON_API_URL: 'https://tlog-api.onrender.com/analyze',
  MAX_FILE_SIZE_MB: 100
};


/**
 * Відкриття Web App
 */
function doGet() {
  return HtmlService
    .createHtmlOutputFromFile('index')
    .setTitle('AI — Аналіз TLOG')
    .setXFrameOptionsMode(
      HtmlService.XFrameOptionsMode.ALLOWALL
    );
}


/**
 * Пробудження Render
 */
function pingServer() {
  try {

    let healthUrl =
      CONFIG.PYTHON_API_URL;


    if (
      healthUrl.endsWith('/analyze')
    ) {

      healthUrl =
        healthUrl.substring(
          0,
          healthUrl.length - 8
        ) + '/health';
    }


    const response =
      UrlFetchApp.fetch(
        healthUrl,
        {
          method: 'get',
          muteHttpExceptions: true
        }
      );


    return (
      response.getResponseCode() === 200
    );

  } catch (e) {

    return false;
  }
}


/**
 * Аналіз TLOG
 */
function analyzeTlog(fileData) {

  const requestId =
    Utilities.getUuid();

  const startTime =
    Date.now();


  try {

    if (!fileData) {
      throw new Error(
        'Файл не отримано.'
      );
    }


    if (!fileData.name) {
      throw new Error(
        'Не вказана назва файлу.'
      );
    }


    if (!fileData.base64) {
      throw new Error(
        'Дані TLOG відсутні.'
      );
    }


    if (!CONFIG.PYTHON_API_URL) {
      throw new Error(
        'Не вказаний PYTHON_API_URL.'
      );
    }


    const bytes =
      Utilities.base64Decode(
        fileData.base64
      );


    const maxSize =
      CONFIG.MAX_FILE_SIZE_MB
      * 1024
      * 1024;


    if (
      bytes.length > maxSize
    ) {

      throw new Error(
        'Файл перевищує '
        + CONFIG.MAX_FILE_SIZE_MB
        + ' MB.'
      );
    }


    const blob =
      Utilities.newBlob(
        bytes,
        'application/octet-stream',
        fileData.name
      );


    const payload = {
      file: blob,
      request_id: requestId,
      original_filename: fileData.name
    };


    const response =
      UrlFetchApp.fetch(
        CONFIG.PYTHON_API_URL,
        {
          method: 'post',
          payload: payload,
          muteHttpExceptions: true,
          followRedirects: true
        }
      );


    const status =
      response.getResponseCode();


    const responseText =
      response.getContentText();


    if (
      status < 200
      || status >= 300
    ) {

      throw new Error(
        'Python API HTTP '
        + status
        + ': '
        + responseText
      );
    }


    let result;


    try {

      result =
        JSON.parse(
          responseText
        );

    } catch (e) {

      throw new Error(
        'Python API повернув не JSON:\n'
        + responseText
      );
    }


    if (!result.debug) {
      result.debug = {};
    }


    result.debug.google = {
      requestId: requestId,
      processingMs:
        Date.now() - startTime
    };


    return result;


  } catch (error) {

    return {
      success: false,
      error:
        error.message
        || error.toString()
    };
  }
}
