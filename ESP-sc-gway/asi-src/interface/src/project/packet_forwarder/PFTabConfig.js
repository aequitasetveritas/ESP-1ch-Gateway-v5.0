import React, { Component } from 'react';


import { ENDPOINT_ROOT } from '../../constants/Env';
import SectionContent from '../../components/SectionContent';
import { restComponent } from '../../components/RestComponent';
import LoadingNotification from '../../components/LoadingNotification';

import { TextValidator, ValidatorForm, SelectValidator } from 'react-material-ui-form-validator';

import Button from '@material-ui/core/Button';
import Typography from '@material-ui/core/Typography';
import Slider from '@material-ui/core/Slider';
import { makeStyles } from '@material-ui/core/styles';
import SaveIcon from '@material-ui/icons/Save';

import MenuItem from '@material-ui/core/MenuItem';
import FormControlLabel from '@material-ui/core/FormControlLabel';
import Switch from '@material-ui/core/Switch';

export const PF_SETTINGS_ENDPOINT = ENDPOINT_ROOT + "packetforwarderSettings"; // RESTful endpoint

const valueToPercentage = (value) => `${Math.round(value / 255 * 100)}%`;

class PFTabConfig extends Component {
  componentDidMount() {
    this.props.loadData();
  }

  render() {
    const { data, fetched, errorMessage, saveData, loadData, handleValueChange, handleSliderChange, handleCheckboxChange} = this.props;
    return (
      <SectionContent title="Configuración de Packet Forwarder" titleGutter>
        <LoadingNotification
          onReset={loadData}
          fetched={fetched}
          errorMessage={errorMessage}
          render={() =>
            <PFConfigForm
              settings={data}
              onReset={loadData}
              onSubmit={saveData}
              handleSliderChange={handleSliderChange}
              handleValueChange={handleValueChange}
              handleCheckboxChange={handleCheckboxChange}
            />
          }
        />
      </SectionContent>
    )
  }
}

const useStyles = makeStyles(theme => ({
  button: {
    marginRight: theme.spacing(2),
    marginTop: theme.spacing(2),
  },
  selectField: {
    width: "100%",
    marginTop: theme.spacing(2),
    marginBottom: theme.spacing(0.5)
  },
  blinkSpeedLabel: {
    marginBottom: theme.spacing(5),
  },
  switchControl: {
    width: "100%",
    marginTop: theme.spacing(2),
    marginBottom: theme.spacing(0.5)
  },
  textField: {
    width: "100%",
    marginTop: theme.spacing(2),
    marginBottom: theme.spacing(0.5)
  },
}));

function PFConfigForm(props) {
  const { settings, onSubmit, onReset, handleValueChange, handleSliderChange, handleCheckboxChange} = props;
  console.log(settings);
  const classes = useStyles();

  var protocoloSelection = settings.protocolo;

  

  return (
    <ValidatorForm onSubmit={onSubmit}>

      <SelectValidator name="protocolo" label="Modo" value={settings.protocolo} className={classes.selectField}
          onChange={handleValueChange('protocolo')}>
          <MenuItem value={1}>Semtech PF - UDP (Solo WiFi)</MenuItem>
          <MenuItem value={2}>Chirpstack Bridge - MQTT/TCP</MenuItem>
        </SelectValidator>

      {settings.protocolo === 1 ? 
        <div>
         <TextValidator
         validators={['required', 'matchRegexp:^.{1,100}$']}
         errorMessages={['Se requiere un host', '100 caracteres max']}
         name="stHost"
         label="Semtech PF Host"
         className={classes.textField}
         value={settings.stHost}
         onChange={handleValueChange('stHost')}
         margin="normal"
         />
         <TextValidator
          validators={['required', 'isNumber', 'minNumber:1025', 'maxNumber:65535']}
          errorMessages={['Es requerido', "Debe ser un número", "Debe ser mayor a 1024 ", "Debe ser menor a 65535"]}
          name="stPort"
          label="Port"
          className={classes.textField}
          value={settings.stPort}
          onChange={handleValueChange('stPort')}
          margin="normal"
          />


        </div>
      :
        <div>
         <TextValidator
          validators={['required', 'matchRegexp:^.{1,100}$']}
          errorMessages={['Se requiere un host', '100 caracteres max']}
          name="mqttHost"
          label="ChirpStack MQTT Host"
          className={classes.textField}
          value={settings.mqttHost}
          onChange={handleValueChange('mqttHost')}
          margin="normal"
          />

           <TextValidator
          validators={['required', 'isNumber', 'minNumber:1025', 'maxNumber:65535']}
          errorMessages={['Es requerido', "Debe ser un número", "Debe ser mayor a 1024 ", "Debe ser menor a 65535"]}
          name="mqttPort"
          label="Port"
          className={classes.textField}
          value={settings.mqttPort}
          onChange={handleValueChange('mqttPort')}
          margin="normal"
          />

             <TextValidator
              validators={['matchRegexp:^.{0,100}$']}
              errorMessages={['Max 100 caracteres']}
              name="mqttUser"
              label="MQTT User"
              className={classes.textField}
              value={settings.mqttUser}
              onChange={handleValueChange('mqttUser')}
              margin="normal"
            />
            <TextValidator
              validators={['matchRegexp:^.{0,32}$']}
              errorMessages={['SSID must be 32 characters or less']}
              name="mqttPass"
              label="MQTT Pass"
              className={classes.textField}
              value={settings.mqttPass}
              onChange={handleValueChange('mqttPass')}
              margin="normal"
            />

        </div>
      }

      <TextValidator
          validators={['required', 'isNumber', 'minNumber:10', 'maxNumber:65535']}
          errorMessages={['Es requerido', "Debe ser un número", "Debe ser mayor a 10 ", "Debe ser menor a 65535"]}
          name="statInterval"
          label="Stats Interval (seg)"
          className={classes.textField}
          value={settings.statInterval}
          onChange={handleValueChange('statInterval')}
          margin="normal"
          />

      <TextValidator
          validators={['required', 'isNumber', 'minNumber:15', 'maxNumber:65535']}
          errorMessages={['Es requerido', "Debe ser un número", "Debe ser mayor a 15 ", "Debe ser menor a 65535"]}
          name="keepAlive"
          label="Keep Alive (mseg)"
          className={classes.textField}
          value={settings.keepAlive}
          onChange={handleValueChange('keepAlive')}
          margin="normal"
          />


      {/*
        <SelectValidator name="sf" label="Spreading Factor" value={loraSettings.sf} className={classes.selectField}
          onChange={handleValueChange('sf')}>
          <MenuItem value={7}>7</MenuItem>
          <MenuItem value={8}>8</MenuItem>
          <MenuItem value={9}>9</MenuItem>
          <MenuItem value={10}>10</MenuItem>
          <MenuItem value={11}>11</MenuItem>
          <MenuItem value={12}>12</MenuItem>
        </SelectValidator>

        <SelectValidator name="frecuencia" label="Frecuencia (Hz)" value={loraSettings.frecuencia} className={classes.selectField}
          onChange={handleValueChange('frecuencia')}>
          <MenuItem value={903900000}>903900000</MenuItem>
          <MenuItem value={915200000}>915200000</MenuItem>
          <MenuItem value={9}>9</MenuItem>
          <MenuItem value={10}>10</MenuItem>
          <MenuItem value={11}>11</MenuItem>
          <MenuItem value={12}>12</MenuItem>
        </SelectValidator>

        <SelectValidator name="backbone" label="Conexión de Datos" value={loraSettings.backbone} className={classes.selectField}
          onChange={handleValueChange('backbone')}>
          <MenuItem value={1}>WiFi</MenuItem>
          <MenuItem value={2}>GPRS</MenuItem>
        </SelectValidator>

        <FormControlLabel className={classes.switchControl}
          control={
            <Switch
              checked={loraSettings.cad}
              onChange={handleCheckboxChange('cad')}
              value="enabled"
              color="primary"
            />
          }
          label="CAD Multi SF (experimental)"
        /> 

        */}

      {/*
      <FormControlLabel className={classes.switchControl}
          control={
            <Switch
              checked={ntpSettings.enabled}
              onChange={handleCheckboxChange('enabled')}
              value="enabled"
              color="primary"
            />
          }
          label="Enable NTP?"
        /> 

      <Slider
        value={demoSettings.blink_speed}
        valueLabelFormat={valueToPercentage}
        aria-labelledby="blink-speed-slider"
        valueLabelDisplay="on"
        min={0}
        max={255}
        onChange={handleSliderChange('blink_speed')}
      />*/}
      <Button startIcon={<SaveIcon />} variant="contained" color="primary" className={classes.button} type="submit">
        Save
      </Button>
      <Button variant="contained" color="secondary" className={classes.button} onClick={onReset}>
        Reset
      </Button>
    </ValidatorForm>
  );
}

export default restComponent(PF_SETTINGS_ENDPOINT, PFTabConfig);
