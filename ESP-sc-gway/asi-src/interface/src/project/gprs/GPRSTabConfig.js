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

export const GPRS_SETTINGS_ENDPOINT = ENDPOINT_ROOT + "gprsSettings"; // RESTful endpoint

const valueToPercentage = (value) => `${Math.round(value / 255 * 100)}%`;

class GPRSTabConfig extends Component {
  componentDidMount() {
    this.props.loadData();
  }

  render() {
    const { data, fetched, errorMessage, saveData, loadData, handleValueChange, handleSliderChange, handleCheckboxChange} = this.props;
    return (
      <SectionContent title="Configuración de GPRS" titleGutter>
        <LoadingNotification
          onReset={loadData}
          fetched={fetched}
          errorMessage={errorMessage}
          render={() =>
            <GPRSConfigForm
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

function GPRSConfigForm(props) {
  const { gprsSettings, onSubmit, onReset, handleValueChange, handleSliderChange, handleCheckboxChange} = props;
  console.log(gprsSettings);
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
      <SelectValidator name="apn" label="APN" value={gprsSettings.apn} className={classes.selectField}
          onChange={handleValueChange('mode_lorawan')}>
          <MenuItem value={true}>internet.datos.peros</MenuItem>
          <MenuItem value={false}>Custom Lora</MenuItem>
        </SelectValidator>

      

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

export default restComponent(GPRS_SETTINGS_ENDPOINT, GPRSTabConfig);
