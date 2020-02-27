import React, { Component } from 'react';


import { ENDPOINT_ROOT } from '../constants/Env';
import SectionContent from '../components/SectionContent';
import { restComponent } from '../components/RestComponent';
import LoadingNotification from '../components/LoadingNotification';

import { TextValidator, ValidatorForm, SelectValidator } from 'react-material-ui-form-validator';

import Button from '@material-ui/core/Button';
import Typography from '@material-ui/core/Typography';
import Slider from '@material-ui/core/Slider';
import { makeStyles } from '@material-ui/core/styles';
import SaveIcon from '@material-ui/icons/Save';

import MenuItem from '@material-ui/core/MenuItem';
import FormControlLabel from '@material-ui/core/FormControlLabel';
import Switch from '@material-ui/core/Switch';

export const LORA_SETTINGS_ENDPOINT = ENDPOINT_ROOT + "loraSettings"; // RESTful endpoint

const valueToPercentage = (value) => `${Math.round(value / 255 * 100)}%`;

class LoraTabConfig extends Component {
  componentDidMount() {
    this.props.loadData();
  }

  render() {
    const { data, fetched, errorMessage, saveData, loadData, handleValueChange, handleSliderChange, handleCheckboxChange} = this.props;
    return (
      <SectionContent title="Configuración de Lora" titleGutter>
        <LoadingNotification
          onReset={loadData}
          fetched={fetched}
          errorMessage={errorMessage}
          render={() =>
            <LoraConfigForm
              loraSettings={data}
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
}));

function LoraConfigForm(props) {
  const { loraSettings, onSubmit, onReset, handleValueChange, handleSliderChange, handleCheckboxChange} = props;
  console.log(loraSettings);
  const classes = useStyles();
  return (
    <ValidatorForm onSubmit={onSubmit}>
      {/*<Typography id="blink-speed-slider" className={classes.blinkSpeedLabel}>
        Blink Speed
      </Typography>*/}

      {/* Modo 
        bool _modo_lorawan;
   uint32_t _frecuencia;
   int _sf;
   int _bw;
   bool _cad;
   uint8_t _backbone;
};
*/}
      <SelectValidator name="mode_lorawan" label="Modo" value={loraSettings.mode_lorawan} className={classes.selectField}
          onChange={handleValueChange('mode_lorawan')}>
          <MenuItem value={true}>LoraWAN 1.0 Compatible</MenuItem>
          <MenuItem value={false}>Custom Lora</MenuItem>
        </SelectValidator>

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

export default restComponent(LORA_SETTINGS_ENDPOINT, LoraTabConfig);
